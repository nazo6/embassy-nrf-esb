//! Low-level radio driver for ESB
//!
//! Usually, you don't need to use this module directly. Instead, use the [`crate::ptx`] or
//! [`crate::prx`] module.

use core::marker::PhantomData;
use core::sync::atomic::{Ordering, compiler_fence};
use core::task::Poll;

use embassy_hal_internal::drop::OnDrop;
use embassy_hal_internal::{PeripheralRef, into_ref};

use embassy_nrf::{
    Peripheral,
    interrupt::{self, typelevel::Interrupt},
    pac::{
        self,
        radio::vals::{self, State as RadioState},
    },
    radio::{Error, Instance, TxPower},
};
use embassy_sync::waitqueue::AtomicWaker;

use crate::addresses::{ADDR_LENGTH, Addresses, address_conversion, bytewise_bit_swap};
use crate::log::{debug, info};
use pid::Pid;

pub mod pid;

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
pub struct Radio<'d, T: Instance, const MAX_PACKET_LEN: usize> {
    _p: PeripheralRef<'d, T>,
    pid_send: Pid,
    pid_recv: Option<Pid>,
    rf_channel: u8,
}

impl<'d, T: Instance, const MAX_PACKET_LEN: usize> Radio<'d, T, MAX_PACKET_LEN> {
    /// Creates a new instance of `Radio`
    ///
    /// **IMPORTANT**: Do **NOT** use peripheral other than [`embassy_nrf::peripherals::RADIO`].
    /// It can cause unexpected behavior.
    pub fn new(
        radio: impl Peripheral<P = T> + 'd,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
        config: RadioConfig,
    ) -> Self {
        const {
            assert!(
                MAX_PACKET_LEN <= 255,
                "MAX_PACKET_LEN must be lower than 256"
            )
        }

        into_ref!(radio);

        let mut radio = Self {
            _p: radio,
            pid_send: Pid::default(),
            pid_recv: None,
            rf_channel: config.addresses.rf_channel,
        };

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

        // Set addresses
        let base0 = address_conversion(u32::from_le_bytes(config.addresses.base0));
        let base1 = address_conversion(u32::from_le_bytes(config.addresses.base1));
        let prefix0 = bytewise_bit_swap(u32::from_le_bytes(config.addresses.prefixes0));
        let prefix1 = bytewise_bit_swap(u32::from_le_bytes(config.addresses.prefixes1));
        r.base0().write(|w| *w = base0);
        r.base1().write(|w| *w = base1);
        r.prefix0().write(|w| w.0 = prefix0);
        r.prefix1().write(|w| w.0 = prefix1);

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

    /// Transmits data
    ///
    /// # Parameters
    /// * `pipe`: Pipe (channel) number
    /// * `buf`: Data to send
    /// * `ack`: Whether ACK is required or not. This is usually false in PRX.
    ///
    /// **NOTE**: If `ack` is true, radio will automatically enter RX mode after send. In such a case, send cannot be executed again unless the recv method is executed!
    pub async fn send(&mut self, pipe: u8, buf: &[u8], ack: bool) -> Result<(), Error> {
        debug!("sending data");

        if buf.len() > MAX_PACKET_LEN - 2 {
            return Err(Error::BufferTooLong);
        }

        let mut temp_buf = [0u8; MAX_PACKET_LEN];
        // LENGTH
        temp_buf[0] = buf.len() as u8;
        // S1 PID (pid is 2bit)
        temp_buf[1] = self.pid_send.inner() << 1;
        self.pid_send.go_next();
        // S1 ack
        temp_buf[1] |= ack as u8;

        temp_buf[2..buf.len() + 2].copy_from_slice(buf);

        let r = self.regs();
        let waker = self.waker();

        self.clear_all_interrupts();

        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
            w.set_disabled_rxen(ack);
        });

        r.intenset().write(|w| w.set_disabled(true));

        r.txaddress().write(|w| w.set_txaddress(pipe));
        r.rxaddresses().write(|w| w.0 = 1 << pipe);

        r.frequency().write(|w| w.set_frequency(self.rf_channel));
        self.set_packet_ptr(&mut temp_buf);

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

            r.intenset().write(|w| w.set_disabled(true));

            Poll::Pending
        })
        .await;

        debug!("sent data");

        Ok(())
    }

    /// Receives data.
    ///
    /// # Parameters
    /// * `enabled_pipes`: Pipes bitmask to receive. For example `0xFF` will enable all pipes.
    /// * `short_tx`: Enables `disabled_txen` shortcut, that means immediately enable TX after recv
    ///   is completed. This is usually used by PRX to send ACK to PTX.
    ///
    /// **NOTE**: If `short_tx` is true, radio will automatically enter TX mode after receive.
    /// In such a case, recv cannot be executed again unless the send method is executed!
    pub async fn recv(
        &mut self,
        enabled_pipes: u8,
        short_tx: bool,
    ) -> Result<(u8, RecvPacket<MAX_PACKET_LEN>), Error> {
        loop {
            if let Some(v) = self.recv_inner(enabled_pipes, short_tx).await? {
                debug!(
                    "Packet received. len:{},pid:{},ack:{}",
                    v.1.payload_length(),
                    v.1.pid(),
                    v.1.ack()
                );
                break Ok(v);
            }
            info!("duplicated packet detected");
        }
    }

    /// Receive data. returns Ok(None) if the packet is duplicated.
    async fn recv_inner(
        &mut self,
        enabled_pipes: u8,
        short_tx: bool,
    ) -> Result<Option<(u8, RecvPacket<MAX_PACKET_LEN>)>, Error> {
        let r = self.regs();
        let waker = self.waker();

        self.clear_all_interrupts();

        r.shorts().write(|w| {
            w.set_ready_start(true);
            w.set_end_disable(true);
            w.set_address_rssistart(true);
            w.set_disabled_rssistop(true);
            w.set_disabled_txen(short_tx);
        });

        r.intenset().write(|w| w.set_disabled(true));

        r.rxaddresses().write(|w| w.0 = enabled_pipes as u32);
        r.frequency().write(|w| w.set_frequency(self.rf_channel));

        let mut temp_buf = [0u8; MAX_PACKET_LEN];
        self.set_packet_ptr(&mut temp_buf);

        r.events_address().write_value(0);
        r.events_payload().write_value(0);
        r.events_disabled().write_value(0);

        // Start receive
        dma_start_fence();
        r.tasks_rxen().write_value(1);

        let dropper = OnDrop::new(|| {
            debug!("receive canceled");
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

        let crc = r.rxcrc().read().rxcrc() as u16;
        if r.crcstatus().read().crcstatus() == vals::Crcstatus::CRCOK {
            let data = RecvPacket(temp_buf);

            if let Some(latest_pid) = self.pid_recv {
                if latest_pid == data.pid() {
                    return Ok(None);
                }
                if !latest_pid.is_next(&data.pid()) {
                    crate::log::debug!("Invalid pid: {}->{}", latest_pid, data.pid());
                }
            }
            self.pid_recv = Some(data.pid());

            Ok(Some((r.rxmatch().read().rxmatch(), data)))
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

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RecvPacket<const N: usize>([u8; N]);

impl<const N: usize> RecvPacket<N> {
    pub fn payload_length(&self) -> u8 {
        self.0[0]
    }

    pub fn payload(&self) -> &[u8] {
        &self.0[2..self.payload_length() as usize + 2]
    }

    pub fn pid(&self) -> Pid {
        Pid::new_unchecked(self.0[1] >> 1)
    }

    pub fn ack(&self) -> bool {
        self.0[1] & 1 != 0
    }
}

/// NOTE must be followed by a volatile write operation
pub(crate) fn dma_start_fence() {
    compiler_fence(Ordering::Release);
}

/// NOTE must be preceded by a volatile read operation
pub(crate) fn dma_end_fence() {
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
