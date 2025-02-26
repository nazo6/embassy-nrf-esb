//! Low-level radio driver for ESB

use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{PeripheralRef, into_ref};

use embassy_nrf::interrupt::typelevel::Interrupt;
use embassy_nrf::interrupt::{self};
use embassy_nrf::pac::radio::vals;
use embassy_nrf::pac::radio::vals::State as RadioState;
use embassy_nrf::radio::{Error, Instance, InterruptHandler, TxPower};
use embassy_nrf::{Peripheral, pac};
use embassy_sync::waitqueue::AtomicWaker;

use crate::addresses::{ADDR_LENGTH, Addresses, address_conversion};

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct RadioConfig {
    /// Tx Power
    pub tx_power: TxPower,
    pub addresses: Addresses,
}

/// Low-level radio
///
/// NOTE: `MAX_PACKET_LEN` is `payload length + 2` bytes (Internally, it is used for S1 and LENGTH)
pub struct Radio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    _p: PeripheralRef<'d, T>,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> Radio<'d, T, MAX_PACKET_LEN> {
    // MAX_PACKET_LEN is actually u8
    const _OK: () = assert!(MAX_PACKET_LEN <= 255);

    /// # Parameters
    /// * max_packet_len: If value is Some, ESB format is DPL (dynamic payload length).
    pub fn new(
        radio: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: RadioConfig,
    ) -> Self {
        into_ref!(radio);

        let mut radio = Self { _p: radio };

        let r = radio.regs();

        // Disable and enable to reset peripheral
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));

        // Enable NRF proprietary 2Mbit mode
        r.mode().write(|w| w.set_mode(vals::Mode::NRF_2MBIT));

        // Set tx_power
        r.txpower().write(|w| w.set_txpower(config.tx_power));

        // Enable shortcuts
        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
        });

        // Set channel (frequency)
        //
        // Safety: It is already checked that the channel is between 0 and 100 in Addresses
        // constructor, so it is safe to cast it to u8
        r.frequency()
            .write(|w| w.set_frequency(config.addresses.rf_channel));

        // Packet config for DPL mode
        r.pcnf0().write(|w| {
            w.set_s0len(false);
            w.set_lflen(if MAX_PACKET_LEN <= 32 { 6 } else { 8 });
            w.set_s1len(3);
        });
        r.pcnf1().write(|w| {
            w.set_whiteen(false);
            w.set_endian(vals::Endian::BIG);
            w.set_balen(ADDR_LENGTH - 1);
            w.set_statlen(0);
            w.set_maxlen(MAX_PACKET_LEN as u8);
        });

        // crc config
        const CRC_INIT: u32 = 0x0000_FFFF;
        const CRC_POLY: u32 = 0x0001_1021;
        r.crcinit().write(|w| w.set_crcinit(CRC_INIT & 0x00FF_FFFF));
        r.crcpoly().write(|w| w.set_crcpoly(CRC_POLY & 0x00FF_FFFF));
        r.crccnf().write(|w| w.set_len(vals::Len::TWO));

        // Set fast ramp-up
        #[cfg(feature = "fast-ru")]
        r.modecnf0().write(|w| w.set_ru(vals::Ru::FAST));

        // Set addresses
        let base0 = address_conversion(u32::from_le_bytes(config.addresses.base0));
        let base1 = address_conversion(u32::from_le_bytes(config.addresses.base1));
        r.base0().write(|w| *w = base0);
        r.base1().write(|w| *w = base1);
        r.prefix0().write(|w| {
            w.set_ap0(config.addresses.prefixes0[0]);
            w.set_ap1(config.addresses.prefixes0[1]);
            w.set_ap2(config.addresses.prefixes0[2]);
            w.set_ap3(config.addresses.prefixes0[3]);
        });
        r.prefix1().write(|w| {
            w.set_ap4(config.addresses.prefixes1[0]);
            w.set_ap5(config.addresses.prefixes1[1]);
            w.set_ap6(config.addresses.prefixes1[2]);
            w.set_ap7(config.addresses.prefixes1[3]);
        });

        // Enable NVIC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        radio
    }

    pub fn regs(&mut self) -> pac::radio::Radio {
        embassy_nrf::pac::RADIO
    }

    fn waker(&mut self) -> &'static AtomicWaker {
        &WAKER
    }

    /// Clear interrupts
    fn clear_all_interrupts(&mut self) {
        self.regs().intenclr().write(|w| w.0 = 0xffff_ffff);
    }

    /// Get the current radio state
    pub fn read_state(&mut self) -> RadioState {
        self.regs().state().read().state()
    }

    // Set pointer to be used for DMA transfer
    //
    // buffer task mutable reference to ensure that the buffer is in RAM.
    fn set_packet_ptr(&mut self, buffer: &mut [u8]) {
        self.regs().packetptr().write_value(buffer.as_ptr() as u32);
    }

    pub async fn send(&mut self, pipe: u8, buf: &[u8], ack: bool) -> Result<(), Error> {
        if buf.len() > MAX_PACKET_LEN - 2 {
            return Err(Error::BufferTooLong);
        }

        let mut temp_buf = [0u8; MAX_PACKET_LEN];
        // LENGTH
        temp_buf[0] = buf.len() as u8;
        // S1 PID (pid is 2bit)
        let pid = 0;
        temp_buf[1] = pid << 1;
        // S1 ack
        temp_buf[1] |= ack as u8;

        temp_buf[2..buf.len() + 2].copy_from_slice(buf);

        let r = self.regs();
        let waker = self.waker();

        self.set_packet_ptr(&mut temp_buf);

        r.txaddress().write(|w| w.set_txaddress(pipe));
        r.rxaddresses().write(|w| w.0 = 1 << pipe);

        self.clear_all_interrupts();
        r.events_address().write_value(0);
        r.events_payload().write_value(0);
        r.events_disabled().write_value(0);

        // Start send
        dma_start_fence();
        r.tasks_txen().write_value(1);
        core::future::poll_fn(|cx| {
            waker.register(cx.waker());

            if r.events_end().read() != 0 {
                r.events_end().write_value(0);
                return Poll::Ready(());
            }

            r.intenset().write(|w| {
                w.set_end(true);
            });

            Poll::Pending
        })
        .await;

        Ok(())
    }

    /// Receive data.
    ///
    /// returns pipe number
    pub async fn recv(
        &mut self,
        enabled_pipes: u32,
    ) -> Result<(u8, RecvPacket<MAX_PACKET_LEN>), Error> {
        let r = self.regs();
        let waker = self.waker();

        r.rxaddresses().write(|w| w.0 = enabled_pipes);

        self.clear_all_interrupts();
        r.events_disabled().write_value(0);

        let mut temp_buf = [0u8; MAX_PACKET_LEN];
        self.set_packet_ptr(&mut temp_buf);

        // Start receive
        dma_start_fence();
        r.tasks_rxen().write_value(1);

        let dropper = OnDrop::new(|| {
            // cancel receive
            r.tasks_stop().write_value(1);
            loop {
                match self.read_state() {
                    RadioState::DISABLED | RadioState::RX_IDLE => break,
                    _ => (),
                }
            }
            dma_end_fence();
        });

        core::future::poll_fn(|cx| {
            waker.register(cx.waker());

            if r.events_end().read() != 0 {
                r.events_phyend().write_value(0);
                // trace!("RX done poll");
                return Poll::Ready(());
            } else {
                r.intenset().write(|w| w.set_phyend(true));
            };

            Poll::Pending
        })
        .await;

        dma_end_fence();
        dropper.defuse();

        let crc = r.rxcrc().read().rxcrc() as u16;
        if r.crcstatus().read().crcstatus() == vals::Crcstatus::CRCOK {
            Ok((r.rxmatch().read().rxmatch(), RecvPacket(temp_buf)))
        } else {
            Err(Error::CrcFailed(crc))
        }
    }

    // Used as workaround of errata 204?
    //
    // /// Waits until the radio state matches the given `state`
    // fn wait_for_radio_state(&mut self, state: RadioState) {
    //     while self.read_state() != state {}
    // }
    //
    // /// Moves the radio from any state to the DISABLED state
    // fn disable(&mut self) {
    //     let r = self.regs();
    //     // See figure 110 in nRF52840-PS
    //     loop {
    //         match self.read_state() {
    //             RadioState::DISABLED => return,
    //             // idle or ramping up
    //             RadioState::RX_RU
    //             | RadioState::RX_IDLE
    //             | RadioState::TX_RU
    //             | RadioState::TX_IDLE => {
    //                 r.tasks_disable().write_value(1);
    //                 self.wait_for_radio_state(RadioState::DISABLED);
    //                 return;
    //             }
    //             // ramping down
    //             RadioState::RX_DISABLE | RadioState::TX_DISABLE => {
    //                 self.wait_for_radio_state(RadioState::DISABLED);
    //                 return;
    //             }
    //             // cancel ongoing transfer
    //             RadioState::RX => {
    //                 r.tasks_stop().write_value(1);
    //                 self.wait_for_radio_state(RadioState::RX_IDLE);
    //             }
    //             RadioState::TX => {
    //                 r.tasks_stop().write_value(1);
    //                 self.wait_for_radio_state(RadioState::TX_IDLE);
    //             }
    //             _ => unreachable!(),
    //         }
    //     }
    // }
}

pub struct RecvPacket<const N: usize>([u8; N]);

impl<const N: usize> RecvPacket<N> {
    pub fn payload_length(&self) -> u8 {
        self.0[0]
    }

    pub fn payload(&self) -> &[u8] {
        &self.0[2..self.payload_length() as usize + 2]
    }

    pub fn pid(&self) -> u8 {
        self.0[1] >> 1
    }

    pub fn ack(&self) -> bool {
        self.0[1] & 1 != 0
    }
}

/// NOTE must be followed by a volatile write operation
pub fn dma_start_fence() {
    compiler_fence(Ordering::Release);
}

/// NOTE must be preceded by a volatile read operation
pub fn dma_end_fence() {
    compiler_fence(Ordering::Acquire);
}

// # Packet
//
// Currently, only DPL mode is implemented.
//
// ## DPL on
// ```
// Packet: | PREAMBLE | BASE   | PREFIX | LENGTH   | S1          | PAYLOAD     | CRC    |
//         | 1byte    | 4bytes | 1byte  | 6or8 bit | 3bit        | LENGTH byte | 2bytes |
// Memory: |          | BASE   | PREFIX | LENGTH   | PID  | ?ACK | PAYLOAD     |        |
//         | ×        | 4bytes | 1byte  | 1byte    | 2bit | 1bit | LENGTH byte | ×      |
//                     <--  ADDRESS  --> <--              DATA               -->
// ```
//
// ## DPL off
// ```
// Packet: | PREAMBLE | BASE   | PREFIX | S0    | S1    | PAYLOAD     | STATLEN | CRC    |
//         | 1byte    | 4bytes | 1byte  | 1byte | 1bit  |             | 1byte   | 2bytes |
// Memory: |          | BASE   | PREFIX | PID   | 0     | PAYLOAD     |         |        |
//         | 0byte    | 4bytes | 1byte  | 1byte | 1byte |             | ×       | ×      |
// ```
