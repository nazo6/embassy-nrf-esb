//! ESB driver for PTX side
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_nrf::{Peripheral, interrupt, radio::Instance};
use embassy_time::Duration;
use nrf_pac::radio::vals::{self, Crcstatus};

use crate::{
    Consumer, Error, GrantW, Producer, Queue, RX_BUF_SIZE, TX_BUF_SIZE,
    log::{debug, error, warn},
    radio::{
        InterruptHandler, Radio, RadioConfig, dma_end_fence, dma_start_fence, packet::Packet,
        pid::Pid,
    },
};

static RX_BUF: Queue<RX_BUF_SIZE> = Queue::new();
// NOTE: byte 0 is used for ack flag (1 = ack), byte 1 is used for pipe select.
static TX_BUF: Queue<TX_BUF_SIZE> = Queue::new();

pub fn new_ptx<T: Instance, const MAX_PACKET_LEN: usize>(
    radio: impl Peripheral<P = T> + 'static,
    irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'static,
    config: RadioConfig,
    ptx_config: PtxConfig,
) -> (PtxTask<T, MAX_PACKET_LEN>, PtxInterface) {
    (
        PtxTask {
            radio: Radio::new(radio, irq, config),
            tx_buf_r: TX_BUF.framed_consumer(),
            rx_buf_w: RX_BUF.framed_producer(),
            config: ptx_config,
        },
        PtxInterface {
            tx_buf_w: TX_BUF.framed_producer(),
            rx_buf_r: RX_BUF.framed_consumer(),
        },
    )
}

/// Config specific to PTX
#[derive(Clone, Debug)]
pub struct PtxConfig {
    pub ack_timeout: Duration,
    pub ack_retransmit_delay: Duration,
    pub ack_retransmit_attempts: u8,
}
impl Default for PtxConfig {
    fn default() -> Self {
        Self {
            ack_timeout: Duration::from_millis(1),
            ack_retransmit_delay: Duration::from_micros(100),
            ack_retransmit_attempts: 3,
        }
    }
}

pub struct PtxTask<T: Instance, const MAX_PACKET_LEN: usize> {
    radio: Radio<'static, T, MAX_PACKET_LEN>,
    tx_buf_r: Consumer<TX_BUF_SIZE>,
    rx_buf_w: Producer<RX_BUF_SIZE>,
    config: PtxConfig,
}

impl<T: Instance, const MAX_PACKET_LEN: usize> PtxTask<T, MAX_PACKET_LEN> {
    pub async fn run(&mut self) {
        let mut pid_send = Pid::default();
        loop {
            match self.run_inner(pid_send).await {
                Ok(_) => {}
                Err(e) => {
                    error!("Send Error: {:?}", e);
                }
            }
            pid_send.go_next();
        }
    }

    async fn run_inner(&mut self, pid: Pid) -> Result<(), Error> {
        let r = self.radio.regs();

        // Receive packet from buffer
        let mut packet = core::pin::pin!(Packet::new_empty());
        let pipe = Self::load_data_to_transmit(&self.tx_buf_r, &mut packet, pid).await?;
        self.radio.set_packet_ptr(&mut packet);

        let ack_required = packet.ack();

        r.txaddress().write(|w| w.set_txaddress(pipe));

        // Make sure that disabled_rxen is disabled even if this function is dropped.

        r.events_disabled().write_value(0);
        r.events_payload().write_value(0);
        r.events_address().write_value(0);

        if ack_required {
            // Create 'backup' packet to retransmit as original packet will be overwritten
            let packet_backup = packet.clone();
            r.rxaddresses().write(|w| w.0 = 1 << pipe as u32);
            'ack_success: {
                for _ in 0..self.config.ack_retransmit_attempts {
                    debug!("Sending packet: {:?}", packet.payload());
                    {
                        let _send_cleanup = OnDrop::new(|| {
                            r.shorts().modify(|w| {
                                w.set_disabled_rxen(false);
                            });
                        });

                        r.shorts().modify(|w| {
                            w.set_disabled_rxen(true);
                        });

                        self.perform_tx(true).await;
                        embassy_time::Timer::after(self.config.ack_timeout).await;
                    }

                    if r.events_end().read() == 1
                        && r.crcstatus().read().crcstatus() == Crcstatus::CRCOK
                    {
                        debug!(
                            "ACK recv: {:?}, status: {:?}",
                            packet.payload(),
                            r.state().read().state()
                        );

                        r.events_end().write_value(0);
                        let _ = Self::save_received_data(&self.rx_buf_w, &packet);
                        break 'ack_success;
                    } else {
                        warn!("ACK recv failed");
                        embassy_time::Timer::after(self.config.ack_retransmit_delay).await;
                        // Failed to receive ack, retransmit
                        self.stop_rx();
                        *packet = packet_backup.clone();
                    }
                }
                warn!("ACK recv gave up");
            }
        } else {
            self.perform_tx(false).await;
        }

        debug!("Send done");
        self.stop_rx();

        Ok(())
    }

    fn stop_rx(&mut self) {
        let r = self.radio.regs();

        loop {
            match r.state().read().state() {
                vals::State::DISABLED => break,
                vals::State::RX_IDLE => {
                    r.tasks_disable().write_value(1);
                }
                _ => {
                    r.tasks_stop().write_value(1);
                }
            }
        }
    }

    async fn perform_tx(&mut self, wait_rx: bool) {
        let r = self.radio.regs();
        let w = self.radio.waker();

        self.radio.clear_all_interrupts();
        r.intenset().write(|w| {
            w.set_disabled(true);
        });
        r.events_disabled().write_value(0);

        dma_start_fence();
        r.tasks_txen().write_value(1);
        core::future::poll_fn(|cx| {
            w.register(cx.waker());
            if r.events_disabled().read() != 0 {
                r.events_disabled().write_value(0);
                r.events_end().write_value(0);
                return Poll::Ready(());
            }

            r.intenset().write(|w| w.set_disabled(true));

            Poll::Pending
        })
        .await;
    }

    async fn load_data_to_transmit(
        tx_buf_r: &Consumer<TX_BUF_SIZE>,
        p: &mut Packet<MAX_PACKET_LEN>,
        pid: Pid,
    ) -> Result<u8, Error> {
        let r = tx_buf_r.wait_read().await;
        if let Err(_e) = p.set_payload(&r[2..r.len()]) {
            warn!("TX payload too big");
            r.release();
            return Err(Error::BufferTooLong);
        }

        let ack = r[0] == 1;
        p.set_ack(ack);
        p.set_pid(pid);

        let pipe = r[1];

        r.release();

        Ok(pipe)
    }

    fn save_received_data(
        rx_buf_w: &Producer<RX_BUF_SIZE>,
        p: &Packet<MAX_PACKET_LEN>,
    ) -> Result<(), Error> {
        if p.payload().is_empty() {
            return Ok(());
        }
        let Ok(mut w) = rx_buf_w.grant(p.payload().len() as u16) else {
            // warn!("RX buffer full");
            return Err(Error::BufferTooLong);
        };
        w.copy_from_slice(p.payload());
        w.commit(p.payload().len() as u16);

        Ok(())
    }
}

pub struct PtxInterface {
    tx_buf_w: Producer<TX_BUF_SIZE>,
    rx_buf_r: Consumer<RX_BUF_SIZE>,
}

impl PtxInterface {
    pub fn try_send(&self, pipe: u8, payload: &[u8], ack: bool) -> Result<(), Error> {
        if let Ok(g) = self.tx_buf_w.grant(payload.len() as u16 + 2) {
            self.send_inner(g, pipe, payload, ack)
        } else {
            Err(Error::BufferTooLong)
        }
    }

    pub async fn send(&self, pipe: u8, payload: &[u8], ack: bool) -> Result<(), Error> {
        let g = self.tx_buf_w.wait_grant(payload.len() as u16 + 2).await;
        self.send_inner(g, pipe, payload, ack)
    }

    fn send_inner<const N: usize>(
        &self,
        mut g: GrantW<N>,
        pipe: u8,
        payload: &[u8],
        ack: bool,
    ) -> Result<(), Error> {
        g[0] = ack as u8;
        g[1] = pipe;
        g[2..2 + payload.len()].copy_from_slice(payload);
        g.commit((payload.len() + 2) as u16);

        Ok(())
    }

    pub async fn recv(&self, buf: &mut [u8]) -> Result<(), Error> {
        let g = self.rx_buf_r.wait_read().await;
        if buf.len() < g.len() {
            return Err(Error::BufferTooShort);
        }

        buf[..g.len()].copy_from_slice(&g);

        Ok(())
    }
}
