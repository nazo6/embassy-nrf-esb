use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_nrf::radio::{Error, Instance};
use nrf_pac::radio::vals::{Crcstatus, State as RadioState};

use crate::log::debug;

use super::{Packet, Radio, dma_end_fence, dma_start_fence, send::RadioSend};

pub struct RadioRecv<'a, 'b, 'd, T: Instance, const MAX_PACKET_LEN: usize> {
    pub(super) radio: &'a mut Radio<'d, T, MAX_PACKET_LEN>,
    pub(crate) packet: &'b mut Packet<MAX_PACKET_LEN>,
}
impl<'a, 'b, 'd, T: Instance, const MAX_PACKET_LEN: usize>
    RadioRecv<'a, 'b, 'd, T, MAX_PACKET_LEN>
{
    pub fn new(
        radio: &'a mut Radio<'d, T, MAX_PACKET_LEN>,
        enabled_pipes: u8,
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
        r.rxaddresses().write(|w| w.0 = enabled_pipes as u32);

        radio.set_packet_ptr(packet);

        Self { radio, packet }
    }

    pub async fn recv(&mut self) -> Result<u8, Error> {
        self.perform_recv().await
    }

    pub fn get_sender(&mut self) -> RadioSend<'_, '_, 'd, T, MAX_PACKET_LEN> {
        RadioSend {
            radio: &mut *self.radio,
            packet: &mut *self.packet,
        }
    }

    async fn perform_recv(&mut self) -> Result<u8, Error> {
        let r = self.radio.regs();
        let w = self.radio.waker();

        self.radio.clear_all_interrupts();

        r.intenset().write(|w| w.set_disabled(true));

        r.events_address().write_value(0);
        r.events_payload().write_value(0);
        r.events_disabled().write_value(0);

        // Start receive
        dma_start_fence();
        r.tasks_rxen().write_value(1);

        let dropper = OnDrop::new(|| {
            // debug!("receive canceled");
            r.tasks_stop().write_value(1);
            loop {
                match self.radio.read_state() {
                    RadioState::DISABLED | RadioState::RX_IDLE => break,
                    _ => (),
                }
            }
            dma_end_fence();
        });

        core::future::poll_fn(|cx| {
            w.register(cx.waker());

            if r.events_disabled().read() != 0 {
                r.events_disabled().write_value(0);
                return Poll::Ready(());
            }

            r.intenset().write(|w| w.set_disabled(true));

            Poll::Pending
        })
        .await;

        dma_end_fence();

        dropper.defuse();

        let pipe = self.check_crc()?;

        debug!(
            "Recv: {:?} (pid:{:?}, ack:{:?})",
            self.packet.payload(),
            self.packet.pid(),
            self.packet.ack()
        );

        Ok(pipe)
    }

    fn check_crc(&mut self) -> Result<u8, Error> {
        let r = self.radio.regs();
        let crc = r.rxcrc().read().rxcrc() as u16;
        if r.crcstatus().read().crcstatus() == Crcstatus::CRCOK {
            Ok(r.rxmatch().read().rxmatch())
        } else {
            Err(Error::CrcFailed(crc))
        }
    }
}
