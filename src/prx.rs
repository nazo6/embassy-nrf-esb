use embassy_nrf::{Peripheral, interrupt, radio::Instance};

use crate::{
    Consumer, Error, InterruptHandler, Producer, Queue, RX_BUF_SIZE, RadioConfig, TX_BUF_SIZE,
    log::{debug, error, warn},
    pid::Pid,
    radio::{Packet, Radio},
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
        let mut latest_recv_pid: Option<Pid> = None;
        let mut latest_sent_pid = Pid::default();

        let mut packet = Packet::new_empty();
        self.radio.set_packet_ptr(&mut packet);

        let mut byte_count = 0;
        let mut loss_count = 0;

        loop {
            self.radio.prepare_recv(0xFF, false);
            match self.radio.perform_recv().await {
                Ok(pipe) => {
                    debug!(
                        "Received data: p:{:?} ack:{:?}",
                        packet.payload(),
                        packet.ack()
                    );

                    byte_count += 1;

                    if let Some(latest_recv_pid) = latest_recv_pid {
                        if latest_recv_pid == packet.pid() {
                            warn!("Duplicate packet received: {:?}", packet.pid());
                            // continue;
                        } else if !packet.pid().is_next_of(&latest_recv_pid) {
                            loss_count += 1;
                            warn!(
                                "Invalid packet order: {} -> {} ({}/{})",
                                latest_recv_pid,
                                packet.pid(),
                                loss_count,
                                byte_count,
                            );
                        }
                    }
                    latest_recv_pid = Some(packet.pid());

                    let ack = packet.ack();
                    let mut g = self
                        .rx_buf_w
                        .wait_grant(packet.payload().len() as u16)
                        .await;
                    g.copy_from_slice(packet.payload());
                    g.commit(packet.payload().len() as u16);

                    if ack {
                        if let Ok(g) = self.tx_buf_r.read() {
                            if let Err(_e) = packet.set_payload(&g) {
                                error!("Payload too big");
                                let _ = packet.set_payload(&[]);
                            }
                            g.release();
                        } else {
                            let _ = packet.set_payload(&[]);
                        }
                        packet.set_ack(false);
                        latest_sent_pid.go_next();
                        packet.set_pid(latest_sent_pid);

                        self.radio.prepare_send(pipe, true);
                        if let Err(e) = self.radio.perform_send().await {
                            warn!("Ack Send error: {:?}", e);
                        }
                        debug!("Ack sent: {:?}", latest_sent_pid);
                    }
                }
                Err(e) => {
                    warn!("Receive error: {:?}", e);
                }
            }
        }
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
