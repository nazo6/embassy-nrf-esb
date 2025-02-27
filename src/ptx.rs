//! ESB driver for PTX side
use embassy_futures::select::{Either, select};
use embassy_nrf::{Peripheral, interrupt, radio::Instance};
use embassy_time::{Duration, Timer};

use crate::{
    Consumer, Error, GrantR, Producer, Queue, RX_BUF_SIZE,
    pid::Pid,
    radio::{InterruptHandler, Packet, Radio, RadioConfig},
};

static BUF: Queue<RX_BUF_SIZE> = Queue::new();

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
            ack_timeout: Duration::from_micros(120),
            ack_retransmit_delay: Duration::from_micros(500),
            ack_retransmit_attempts: 3,
        }
    }
}

pub struct PtxRadio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    radio: Radio<'d, T, MAX_PACKET_LEN>,
    rx_buf_w: Producer<RX_BUF_SIZE>,
    rx_buf_r: Consumer<RX_BUF_SIZE>,
    config: PtxConfig,
    last_sent_pid: Pid,
    last_recv_pid: Option<Pid>,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> PtxRadio<'d, T, MAX_PACKET_LEN> {
    pub fn new(
        p: impl Peripheral<P = T> + 'd,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        radio_config: RadioConfig,
        ptx_config: PtxConfig,
    ) -> Result<Self, Error> {
        Ok(Self {
            radio: Radio::new(p, irq, radio_config),
            rx_buf_w: BUF.framed_producer(),
            rx_buf_r: BUF.framed_consumer(),
            config: ptx_config,
            last_sent_pid: Pid::default(),
            last_recv_pid: None,
        })
    }

    /// Send data to PRX.
    ///
    /// If ack packet from PRX has payload, it will be stored in internal FIFO buffer. This data can be read by calling `recv_dequeue`.
    pub async fn send(&mut self, pipe: u8, buf: &[u8], ack: bool) -> Result<(), Error> {
        self.last_sent_pid.go_next();
        let mut packet = match Packet::new(buf, self.last_sent_pid, ack) {
            Ok(p) => p,
            Err(e) => return Err(Error::Send(e)),
        };

        self.radio
            .send(pipe, &mut packet)
            .await
            .map_err(Error::Send)?;

        'ack: {
            if ack {
                for _i in 0..self.config.ack_retransmit_attempts {
                    let c = self.config.clone();
                    match select(self.recv_ack(pipe), Timer::after(c.ack_timeout)).await {
                        Either::First(Ok(())) => {
                            break 'ack;
                        }
                        Either::First(Err(e)) => {
                            return Err(e);
                        }
                        _ => {
                            Timer::after(c.ack_retransmit_delay).await;
                            self.last_sent_pid.go_next();
                            self.radio
                                .send(pipe, &mut packet)
                                .await
                                .map_err(Error::Send)?;
                        }
                    }
                }

                return Err(Error::AckTimeout);
            }
        }

        Ok(())
    }

    /// Receive ack paylod data sent from PRX.
    ///
    /// This data is received when sending data and is stored in internal FIFO buffer.
    ///
    /// NOTE: Make sure to call release on the grant after processing the data.
    pub fn recv_dequeue(&mut self) -> impl Iterator<Item = GrantR<'static, RX_BUF_SIZE>> {
        core::iter::from_fn(|| self.rx_buf_r.read().ok())
    }

    // Receive ack and save to fifo
    async fn recv_ack(&mut self, pipe: u8) -> Result<(), Error> {
        let (recv_pipe, packet) = self
            .radio
            .recv_once(1 << pipe, false)
            .await
            .map_err(Error::Recv)?;

        if pipe != recv_pipe {
            return Err(Error::InvalidAck);
        }

        if let Some(latest_pid) = self.last_recv_pid {
            // pid was same as latest packet. PRX should have received the packet
            if latest_pid == packet.pid() {
                return Ok(());
            }
        }
        if !packet.payload().is_empty() {
            self.last_recv_pid = Some(packet.pid());
            let mut g = self
                .rx_buf_w
                .grant(packet.payload().len() as u16)
                .map_err(|_| Error::BufferTooLong)?;
            g.copy_from_slice(packet.payload());
            g.commit(packet.payload().len() as u16);
        }

        Ok(())
    }
}
