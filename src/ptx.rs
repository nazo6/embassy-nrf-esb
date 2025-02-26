use bbqueue::{BBBuffer, framed::FrameGrantR};
use embassy_nrf::{
    Peripheral, interrupt,
    radio::{Instance, InterruptHandler},
};

use crate::{
    Error,
    radio::{Radio, RadioConfig},
};

const FIFO_SIZE: usize = 1024;

static BUF: BBBuffer<FIFO_SIZE> = BBBuffer::new();

pub struct PtxRadio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    radio: Radio<'d, T, MAX_PACKET_LEN>,
    rx_buf_w: bbqueue::framed::FrameProducer<'static, FIFO_SIZE>,
    rx_buf_r: bbqueue::framed::FrameConsumer<'static, FIFO_SIZE>,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> PtxRadio<'d, T, MAX_PACKET_LEN> {
    pub fn new(
        p: impl Peripheral<P = T> + 'd,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        radio_config: RadioConfig,
    ) -> Result<Self, Error> {
        let (rx_buf_w, rx_buf_r) = BUF
            .try_split_framed()
            .map_err(|_| Error::AlreadyInitialized)?;

        Ok(Self {
            radio: Radio::new(p, irq, radio_config),
            rx_buf_w,
            rx_buf_r,
        })
    }

    /// Send data to PRX. If ack packet from PRX has payload, it will be stored in internal FIFO
    /// buffer. This data can be read by calling `recv_dequeue`.
    pub async fn send(&mut self, pipe: u8, buf: &mut [u8], ack: bool) -> Result<(), Error> {
        let r = self.radio.regs();
        if ack {
            // Start RX after TX if ack is expected
            r.shorts().write(|w| w.set_disabled_rxen(true));
        }

        self.radio.send(pipe, buf, ack).await.map_err(Error::Send)?;

        if ack {
            self.recv_ack(pipe).await?;
        }

        Ok(())
    }

    /// Receive data sent from PRX. This data is received when sending data and is stored in internal FIFO buffer.
    pub fn recv_dequeue(&mut self) -> impl Iterator<Item = FrameGrantR<'static, FIFO_SIZE>> {
        core::iter::from_fn(|| {
            if let Some(mut frame) = self.rx_buf_r.read() {
                frame.auto_release(true);
                Some(frame)
            } else {
                None
            }
        })
    }

    // Receive ack and save to fifo
    async fn recv_ack(&mut self, pipe: u8) -> Result<(), Error> {
        let (recv_pipe, packet) = self.radio.recv().await.map_err(Error::Recv)?;
        if pipe != recv_pipe {
            return Err(Error::InvalidAck);
        }

        let mut g = self
            .rx_buf_w
            .grant(packet.payload_length() as usize)
            .map_err(|_| Error::BufferTooLong)?;
        g.copy_from_slice(packet.payload());
        g.commit(packet.payload_length() as usize);

        Ok(())
    }
}
