//! ESB driver for PTX side
use embassy_nrf::{Peripheral, interrupt, radio::Instance};
use embassy_time::Duration;

use crate::{
    Consumer, Error, GrantW, Producer, Queue, RX_BUF_SIZE, TX_BUF_SIZE,
    log::{debug, error, warn},
    pid::Pid,
    radio::{InterruptHandler, Packet, Radio, RadioConfig},
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
            ack_timeout: Duration::from_millis(100),
            ack_retransmit_delay: Duration::from_micros(500),
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

        let mut packet = Packet::new_empty();
        self.radio.set_packet_ptr(&mut packet);
        loop {
            let r = self.tx_buf_r.wait_read().await;
            if let Err(_e) = packet.set_payload(&r[2..r.len()]) {
                error!("Payload too long");
                continue;
            }

            let ack = r[0] == 1;
            packet.set_ack(ack);

            pid_send.go_next();
            packet.set_pid(pid_send);

            let pipe = r[1];

            r.release();

            self.radio.prepare_send(pipe, ack);
            if let Err(e) = self.radio.perform_send().await {
                warn!("Send failed: {:?}", e);
            }

            if ack {
                self.radio.prepare_recv(1 << pipe, false);
                match self.radio.perform_recv().await {
                    Ok(_addr) => {
                        debug!("Received ACK: {:?}", packet.payload());
                        let mut w = self
                            .rx_buf_w
                            .wait_grant(packet.payload().len() as u16)
                            .await;
                        w.copy_from_slice(packet.payload());
                        w.commit(packet.payload().len() as u16);
                    }
                    Err(e) => {
                        warn!("ACK Recv failed: {:?}", e);
                        continue;
                    }
                }
                debug!("ACK OK");
            }
        }
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
