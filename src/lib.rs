#![no_std]

pub mod addresses;
pub mod config;
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
}
