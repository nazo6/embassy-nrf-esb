use embassy_nrf::{Peripheral, interrupt, radio::Instance};

use crate::{
    Consumer, Error, InterruptHandler, Producer, Queue, RX_BUF_SIZE, RadioConfig, TX_BUF_SIZE,
    log::{debug, error, warn},
    radio::{Radio, packet::Packet, pid::Pid},
};

static TX_BUF: Queue<TX_BUF_SIZE> = Queue::new();
static RX_BUF: Queue<RX_BUF_SIZE> = Queue::new();

pub fn new_prx<T: Instance, const MAX_PACKET_LEN: usize>(
    radio: impl Peripheral<P = T> + 'static,
    irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'static,
    config: RadioConfig,
) -> (PrxTask<T, MAX_PACKET_LEN>, PrxInterface) {
    (
        PrxTask {
            radio: Radio::new(radio, irq, config),
            tx_buf_r: TX_BUF.framed_consumer(),
            rx_buf_w: RX_BUF.framed_producer(),
        },
        PrxInterface {
            tx_buf_w: TX_BUF.framed_producer(),
            rx_buf_r: RX_BUF.framed_consumer(),
        },
    )
}

pub struct PrxTask<T: Instance, const MAX_PACKET_LEN: usize> {
    radio: Radio<'static, T, MAX_PACKET_LEN>,
    tx_buf_r: Consumer<TX_BUF_SIZE>,
    rx_buf_w: Producer<RX_BUF_SIZE>,
}

impl<T: Instance, const MAX_PACKET_LEN: usize> PrxTask<T, MAX_PACKET_LEN> {
    pub async fn run(&mut self) {
        let r = self.radio.regs();

        let mut latest_recv_pid: Option<Pid> = None;

        let mut packet = core::pin::pin!(Packet::new_empty());

        let mut byte_count = 0;
        let mut loss_count = 0;

        r.rxaddresses().write(|w| w.0 = 0xFF);
        self.radio.set_packet_ptr(&mut packet);

        loop {
            r.shorts().modify(|w| w.set_disabled_txen(true));
            let res = self.radio.perform_rx().await;
            r.shorts().modify(|w| w.set_disabled_txen(false));
            match res {
                Ok(pipe) => {
                    let recv_pid = packet.pid();
                    let ack = packet.ack();

                    byte_count += 1;

                    if let Some(latest_recv_pid) = latest_recv_pid {
                        if latest_recv_pid == recv_pid {
                            warn!("Duplicate packet received: {:?}", recv_pid);
                            // continue;
                        } else if !packet.pid().is_next_of(&latest_recv_pid) {
                            loss_count += 1;
                            warn!(
                                "Invalid packet order: {} -> {} ({}/{})",
                                latest_recv_pid, recv_pid, loss_count, byte_count,
                            );
                        }
                    }
                    latest_recv_pid = Some(recv_pid);
                    if let Err(e) = Self::save_received_data(&self.rx_buf_w, &*packet) {
                        // todo
                    }

                    if ack {
                        r.txaddress().write(|w| w.set_txaddress(pipe));
                        Self::load_data_to_transmit(&self.tx_buf_r, &mut *packet, recv_pid);
                        self.radio.perform_tx().await;
                        debug!("PRX: Sent ACK packet: {:?}", packet.payload());
                    }
                }
                Err(e) => {
                    warn!("Receive error: {:?}", e);
                }
            }
        }
    }

    fn load_data_to_transmit(
        tx_buf_r: &Consumer<TX_BUF_SIZE>,
        p: &mut Packet<MAX_PACKET_LEN>,
        pid: Pid,
    ) {
        if let Ok(g) = tx_buf_r.read() {
            if let Err(_e) = p.set_payload(&g) {
                error!("Payload too big");
                let _ = p.set_payload(&[0, 1, 2]);
            }
            g.release();
        } else {
            let _ = p.set_payload(&[2, 1, 0]);
        }
        p.set_ack(false);
        p.set_pid(pid);
    }

    fn save_received_data(
        rx_buf_w: &Producer<RX_BUF_SIZE>,
        p: &Packet<MAX_PACKET_LEN>,
    ) -> Result<(), Error> {
        let Ok(mut g) = rx_buf_w.grant(p.payload().len() as u16) else {
            return Err(Error::BufferTooLong);
        };
        g.copy_from_slice(p.payload());
        g.commit(p.payload().len() as u16);
        Ok(())
    }
}

pub struct PrxInterface {
    tx_buf_w: Producer<TX_BUF_SIZE>,
    rx_buf_r: Consumer<RX_BUF_SIZE>,
}

impl PrxInterface {
    pub async fn recv(&mut self, buf: &mut [u8]) -> Result<usize, Error> {
        let g = self.rx_buf_r.wait_read().await;
        let len = g.len();
        if buf.len() < len {
            return Err(Error::BufferTooShort);
        }

        buf[..len].copy_from_slice(&g);

        g.release();

        Ok(len)
    }

    pub async fn send(&mut self, buf: &[u8]) {
        let mut g = self.tx_buf_w.wait_grant(buf.len() as u16).await;
        g.copy_from_slice(buf);
        g.commit(buf.len() as u16);
    }
}
