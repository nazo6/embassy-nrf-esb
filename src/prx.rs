//! ESB driver for PRX side
//!
//! PRX listens data from PTX. The connection is started by PTX.

use bbqueue::BBBuffer;
use embassy_nrf::{Peripheral, interrupt, radio::Instance};

use crate::{
    Error,
    radio::{InterruptHandler, Radio, RadioConfig},
};

const BUF_SIZE: usize = 1024;

static BUF: BBBuffer<BUF_SIZE> = BBBuffer::new();

pub struct PrxRadio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    radio: Radio<'d, T, MAX_PACKET_LEN>,
    tx_buf_w: bbqueue::framed::FrameProducer<'static, BUF_SIZE>,
    tx_buf_r: bbqueue::framed::FrameConsumer<'static, BUF_SIZE>,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> PrxRadio<'d, T, MAX_PACKET_LEN> {
    pub fn new(
        p: impl Peripheral<P = T> + 'd,
        irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        radio_config: RadioConfig,
    ) -> Result<Self, Error> {
        let (tx_buf_w, tx_buf_r) = BUF
            .try_split_framed()
            .map_err(|_| Error::AlreadyInitialized)?;

        Ok(Self {
            radio: Radio::new(p, irq, radio_config),
            tx_buf_w,
            tx_buf_r,
        })
    }

    /// Enqueue data to be sent with ack packet
    ///
    /// To send data, call `recv`.
    pub fn send_enqueue(&mut self, data: &[u8]) -> Result<(), Error> {
        let mut g = self
            .tx_buf_w
            .grant(data.len())
            .map_err(|_| Error::BufferTooLong)?;
        g.copy_from_slice(data);
        g.commit(data.len());

        Ok(())
    }

    /// Wait for received data from PTX. Also, enqueued data will be sent to PTX.
    ///
    /// Returns the number of bytes received.
    pub async fn recv(&mut self, buf: &mut [u8], enabled_pipes: u8) -> Result<usize, Error> {
        let (recv_pipe, packet) = self
            .radio
            .recv(enabled_pipes, false)
            .await
            .map_err(Error::Recv)?;

        if buf.len() < packet.payload_length() as usize {
            return Err(Error::BufferTooShort);
        }
        buf[..packet.payload_length() as usize].copy_from_slice(packet.payload());

        if packet.ack() {
            self.send_ack(recv_pipe).await.map_err(Error::Send)?;
        }

        Ok(packet.payload_length() as usize)
    }

    async fn send_ack(&mut self, pipe: u8) -> Result<(), embassy_nrf::radio::Error> {
        if let Some(frame) = self.tx_buf_r.read() {
            self.radio.send(pipe, &frame, false).await?;
            frame.release();
        } else {
            self.radio.send(pipe, &[], false).await?;
        }
        Ok(())
    }
}
