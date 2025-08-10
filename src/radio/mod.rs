//! Low-level radio driver for ESB
//!
//! Usually, you don't need to use this module directly. Instead, use the [`crate::ptx`] or
//! [`crate::prx`] module.

use core::marker::PhantomData;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_nrf::radio::Error as RadioError;
use embassy_nrf::{
    Peri,
    interrupt::{self, typelevel::Interrupt},
    pac::{
        self,
        radio::vals::{self, State as RadioState},
    },
    radio::{Instance, TxPower},
};
use embassy_sync::waitqueue::AtomicWaker;
use packet::Packet;

use crate::Error;
use crate::addresses::{ADDR_LENGTH, Addresses, address_conversion, bytewise_bit_swap};

pub(crate) mod packet;
pub(crate) mod pid;

static WAKER: AtomicWaker = AtomicWaker::new();

pub struct InterruptHandler<T: Instance> {
    _phantom: PhantomData<T>,
}

// FIXME: Here, hard-coaded RADIO is used as the value because `radio::Instance::regs()`
// is not publicly available and cannot be used.
// Probably this should not be a problem since there are no nRF devices with radio peripherals other than RADIO,
// but if T is not RADIO this can be unsound.
impl<T: Instance> interrupt::typelevel::Handler<T::Interrupt> for InterruptHandler<T> {
    unsafe fn on_interrupt() {
        embassy_nrf::pac::RADIO
            .intenclr()
            .write(|w| w.0 = 0xffff_ffff);
        WAKER.wake();
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RadioConfig {
    /// Tx Power
    pub tx_power: TxPower,
    pub addresses: Addresses,
}

impl Default for RadioConfig {
    fn default() -> Self {
        Self {
            tx_power: TxPower::_0_DBM,
            addresses: Addresses::default(),
        }
    }
}

/// Low-level radio
///
/// NOTE: `MAX_PACKET_LEN` is `payload length + 2` bytes (Internally, it is used for S1 and LENGTH)
pub(crate) struct Radio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    _p: Peri<'d, T>,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> Radio<'d, T, MAX_PACKET_LEN> {
    /// Creates a new instance of `Radio`
    ///
    /// **IMPORTANT**: Do **NOT** use peripheral other than [`embassy_nrf::peripherals::RADIO`].
    /// It can cause unexpected behavior.
    pub fn new(
        radio: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: RadioConfig,
    ) -> Self {
        const {
            assert!(
                MAX_PACKET_LEN <= 255,
                "MAX_PACKET_LEN must be lower than 256"
            );
            assert!(
                MAX_PACKET_LEN >= 3,
                "MAX_PACKET_LEN must be greater than or equal to 3"
            )
        }

        let mut radio = Self { _p: radio };

        let r = radio.regs();

        // Disable and enable to reset peripheral
        r.power().write(|w| w.set_power(false));
        r.power().write(|w| w.set_power(true));

        // Enable NRF proprietary 2Mbit mode
        r.mode().write(|w| w.set_mode(vals::Mode::NRF_2MBIT));

        // Set fast ramp-up
        #[cfg(feature = "fast-ru")]
        r.modecnf0().write(|w| w.set_ru(vals::Ru::FAST));

        // Set tx_power
        r.txpower().write(|w| w.set_txpower(config.tx_power));

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
            w.set_whiteen(true);
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

        // Set addresses
        let base0 = address_conversion(u32::from_le_bytes(config.addresses.base0));
        let base1 = address_conversion(u32::from_le_bytes(config.addresses.base1));
        let prefix0 = bytewise_bit_swap(u32::from_le_bytes(config.addresses.prefixes0));
        let prefix1 = bytewise_bit_swap(u32::from_le_bytes(config.addresses.prefixes1));
        r.base0().write(|w| *w = base0);
        r.base1().write(|w| *w = base1);
        r.prefix0().write(|w| w.0 = prefix0);
        r.prefix1().write(|w| w.0 = prefix1);

        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
        });

        // Enable NVIC interrupt
        T::Interrupt::unpend();
        unsafe { T::Interrupt::enable() };

        radio
    }

    // FIXME: Same as above, hard-coded RADIO is used.
    pub fn regs(&mut self) -> pac::radio::Radio {
        embassy_nrf::pac::RADIO
    }

    pub fn waker(&mut self) -> &'static AtomicWaker {
        &WAKER
    }

    /// Clear interrupts
    pub fn clear_all_interrupts(&mut self) {
        self.regs().intenclr().write(|w| w.0 = 0xffff_ffff);
    }

    /// Get the current radio state
    pub fn read_state(&mut self) -> RadioState {
        self.regs().state().read().state()
    }

    // Set pointer to be used for DMA transfer
    //
    // buffer task mutable reference to ensure that the buffer is in RAM.
    pub fn set_packet_ptr(&mut self, packet: &mut Packet<MAX_PACKET_LEN>) {
        self.regs()
            .packetptr()
            .write_value(packet.0.as_ptr() as u32);
    }

    pub async fn perform_tx(&mut self) {
        let r = self.regs();
        let w = self.waker();

        self.clear_all_interrupts();
        r.intenset().write(|w| w.set_disabled(true));
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

    pub async fn perform_rx(&mut self) -> Result<u8, Error> {
        let r = self.regs();
        let w = self.waker();

        self.clear_all_interrupts();

        r.intenset().write(|w| w.set_disabled(true));

        r.events_address().write_value(0);
        r.events_payload().write_value(0);
        r.events_disabled().write_value(0);

        // Start receive
        dma_start_fence();
        r.tasks_rxen().write_value(1);

        let dropper = OnDrop::new(|| {
            r.tasks_stop().write_value(1);
            loop {
                match self.read_state() {
                    vals::State::DISABLED | vals::State::RX_IDLE => break,
                    _ => (),
                }
            }
            dma_end_fence();
        });

        core::future::poll_fn(|cx| {
            defmt::debug!("rx wake");

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

        let pipe = self.check_crc().map_err(Error::Recv)?;

        Ok(pipe)
    }

    fn check_crc(&mut self) -> Result<u8, RadioError> {
        let r = self.regs();
        let crc = r.rxcrc().read().rxcrc() as u16;
        if r.crcstatus().read().crcstatus() == vals::Crcstatus::CRCOK {
            Ok(r.rxmatch().read().rxmatch())
        } else {
            Err(RadioError::CrcFailed(crc))
        }
    }
}

/// NOTE must be followed by a volatile write operation
pub(crate) fn dma_start_fence() {
    compiler_fence(Ordering::Release);
    core::sync::atomic::fence(Ordering::SeqCst);
}

/// NOTE must be preceded by a volatile read operation
pub(crate) fn dma_end_fence() {
    compiler_fence(Ordering::Acquire);
    core::sync::atomic::fence(Ordering::SeqCst);
}
