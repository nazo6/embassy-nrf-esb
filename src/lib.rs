#![no_std]
//! This library is async implementation of the [Enhanced ShockBurst (ESB) protocol](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/esb/index.html).
//!
//! The ESB consists of two roles called PTX and PRX; communication is always initiated from PTX.
//! The PTX can define that an ACK packet should be returned by the PRX.
//! In such a case, the PRX receives data from the PTX and returns an ACK packet to the PTX.
//! If no ACK packet is sent, the PTX can resend the data.
//!
//! In addition, the ACK packet can contain additional payload. This allows two-way communication between the PTX and PRX.
//!
//! ## Note about `MAX_PACKET_LEN`
//! Note that `MAX_PACKET_LEN` is the maximum length of the **packet**.
//! What is often called “data” is the payload, which is two bytes shorter than the packet.
//! In other words, `MAX_PACKET_LEN` should be 2 bytes larger than the maximum size of the data you wish to send.

pub mod addresses;
mod log;
pub mod prx;
pub mod ptx;
pub mod radio;

pub use radio::InterruptHandler;
pub use radio::RadioConfig;

/// Size of the FIFO buffer used to store data to be sent or received.
///
/// TODO: This should be configurable.
pub const FIFO_SIZE: usize = 1024;

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
