#![no_std]

pub mod addresses;
pub mod prx;
pub mod ptx;
mod radio;

#[derive(Debug, Clone)]
pub enum Error {
    AlreadyInitialized,
    Recv(embassy_nrf::radio::Error),
    Send(embassy_nrf::radio::Error),
    BufferTooLong,
    BufferTooShort,
    InvalidAck,
    AckTimeout,
}

pub use radio::RadioConfig;

mod pid {
    /// PID is 2 bit number.
    #[derive(Default, PartialEq, Eq, Debug, Clone, Copy)]
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
