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
mod radio;

use bbq2::prod_cons::framed::FramedConsumer;
use bbq2::prod_cons::framed::FramedGrantW;
use bbq2::prod_cons::framed::FramedProducer;
use bbq2::queue::BBQueue;
use bbq2::traits::coordination::cas::AtomicCoord;
use bbq2::traits::notifier::maitake::MaiNotSpsc;
use bbq2::traits::storage::Inline;
pub use radio::InterruptHandler;
pub use radio::RadioConfig;

const TX_BUF_SIZE: usize = 256;
const RX_BUF_SIZE: usize = 256;

type Producer<const N: usize> = FramedProducer<
    &'static BBQueue<Inline<N>, AtomicCoord, MaiNotSpsc>,
    Inline<N>,
    AtomicCoord,
    MaiNotSpsc,
>;

type Consumer<const N: usize> = FramedConsumer<
    &'static BBQueue<Inline<N>, AtomicCoord, MaiNotSpsc>,
    Inline<N>,
    AtomicCoord,
    MaiNotSpsc,
>;
type Queue<const N: usize> = BBQueue<Inline<N>, AtomicCoord, MaiNotSpsc>;

// type GrantR<'a, const N: usize> = FramedGrantR<&'a Queue<N>, Inline<N>, AtomicCoord, MaiNotSpsc>;
type GrantW<'a, const N: usize> = FramedGrantW<&'a Queue<N>, Inline<N>, AtomicCoord, MaiNotSpsc>;

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
