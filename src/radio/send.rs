use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_nrf::radio::{Error, Instance};

use crate::log::debug;

use super::{Packet, Radio, dma_start_fence, recv::RadioRecv};

pub struct RadioSend<'a, 'b, 'd, T: Instance, const MAX_PACKET_LEN: usize> {
    pub(crate) radio: &'a mut Radio<'d, T, MAX_PACKET_LEN>,
    pub(crate) packet: &'b mut Packet<MAX_PACKET_LEN>,
}
impl<'a, 'b, 'd, T: Instance, const MAX_PACKET_LEN: usize>
    RadioSend<'a, 'b, 'd, T, MAX_PACKET_LEN>
{
    pub fn new(
        radio: &'a mut Radio<'d, T, MAX_PACKET_LEN>,
        send_pipe: u8,
        packet: &'b mut Packet<MAX_PACKET_LEN>,
    ) -> Self {
        let r = radio.regs();

        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
        });

        r.frequency().write(|w| w.set_frequency(radio.rf_channel));
        r.txaddress().write(|w| w.set_txaddress(send_pipe));

        radio.set_packet_ptr(packet);

        Self { radio, packet }
    }

    /// Send data in `packet` and immediately receive ack and save payload to `packet`.
    ///
    /// # Returns
    /// * Ok(pipe): The pipe number that received the packet.
    pub async fn send_and_recv(
        &mut self,
    ) -> Result<RadioRecv<'_, '_, 'd, T, MAX_PACKET_LEN>, Error> {
        let r = self.radio.regs();
        r.shorts().modify(|w| {
            w.set_disabled_rxen(true);
        });

        r.rxaddresses()
            .write(|w| w.0 = 1 << r.txaddress().read().txaddress() as u32);

        let _dropper = OnDrop::new(|| {
            r.shorts().modify(|w| {
                w.set_disabled_rxen(false);
            });
        });

        self.radio.clear_all_interrupts();
        self.perform_send().await?;

        // Safety: It is safe to initialize RadioRecv without initializer because this function
        // already set the radio correctly.
        let recv = RadioRecv {
            radio: self.radio,
            packet: self.packet,
        };
        Ok(recv)
    }

    pub async fn send(&mut self) -> Result<(), Error> {
        self.perform_send().await
    }

    async fn perform_send(&mut self) -> Result<(), Error> {
        let r = self.radio.regs();
        let waker = self.radio.waker();

        self.radio.clear_all_interrupts();

        r.intenset().write(|w| w.set_disabled(true));

        r.events_disabled().write_value(0);

        // Start send
        dma_start_fence();
        r.tasks_txen().write_value(1);
        core::future::poll_fn(|cx| {
            waker.register(cx.waker());
            if r.events_disabled().read() != 0 {
                r.events_disabled().write_value(0);
                return Poll::Ready(());
            }

            r.intenset().write(|w| w.set_disabled(true));

            Poll::Pending
        })
        .await;

        debug!(
            "Sent: {:?} (pid:{:?}, ack:{:?})",
            self.packet.payload(),
            self.packet.pid(),
            self.packet.ack()
        );

        Ok(())
    }
}
