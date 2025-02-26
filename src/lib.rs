#![no_std]

pub mod addresses;
pub mod prx;
pub mod ptx;
mod radio;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    AlreadyInitialized,
    Recv(embassy_nrf::radio::Error),
    Send(embassy_nrf::radio::Error),
    BufferTooLong,
    BufferTooShort,
    InvalidAck,
    AckTimeout,
}

pub use radio::InterruptHandler;
pub use radio::RadioConfig;

mod pid {
    /// PID is 2 bit number.
    #[derive(Default, PartialEq, Eq, Debug, Clone, Copy)]
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct Pid(u8);
    impl Pid {
        pub fn new_unchecked(val: u8) -> Self {
            Self(val)
        }

        pub fn inner(&self) -> u8 {
            self.0
        }

        pub fn go_next(&mut self) {
            if self.0 == 0b11 {
                self.0 = 0;
            } else {
                self.0 += 1;
            }
        }
    }
}

mod log {
    macro_rules! debug {
        ($($arg:tt)*) => {
            #[cfg(feature = "defmt")]
            {
                defmt::debug!($($arg)*);
            }
        }
    }
    pub(crate) use debug;

    macro_rules! info {
        ($($arg:tt)*) => {
            #[cfg(feature = "defmt")]
            {
                defmt::info!($($arg)*);
            }
        }
    }
    pub(crate) use info;

    macro_rules! warni {
        ($($arg:tt)*) => {
            #[cfg(feature = "defmt")]
            {
                defmt::warn!($($arg)*);
            }
        }
    }
    pub(crate) use warni as warn;

    macro_rules! error {
        ($($arg:tt)*) => {
            #[cfg(feature = "defmt")]
            {
                defmt::error!($($arg)*);
            }
        }
    }
    pub(crate) use error;
}
