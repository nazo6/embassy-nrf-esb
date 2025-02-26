use embassy_nrf::radio::TxPower;

// TODO: Figure it out good values
const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u16 = 120;
const RETRANSMIT_DELAY_US_OFFSET: u16 = 62;
const RETRANSMIT_DELAY: u16 = 500;
const MAXIMUM_TRANSMIT_ATTEMPTS: u8 = 3;
const ENABLED_PIPES: u8 = 0xFF;

#[cfg(not(feature = "fast-ru"))]
pub(crate) const RAMP_UP_TIME: u16 = 140;

// This is only true if we enable the fast ramp-up time, which we do
#[cfg(feature = "fast-ru")]
pub(crate) const RAMP_UP_TIME: u16 = 40;

/// Protocol configuration
#[derive(Copy, Clone)]
pub struct Config {
    /// Number of microseconds to wait for an acknowledgement before timing out
    pub wait_for_ack_timeout: u16,
    /// Delay, in microseconds, between retransmissions when the radio does not receive an
    /// acknowledgement
    pub retransmit_delay: u16,
    /// Maximum number of transmit attempts when an acknowledgement is not received
    pub maximum_transmit_attempts: u8,
    /// A bit mask representing the pipes that the radio must listen while receiving, the LSb is
    /// pipe zero
    pub enabled_pipes: u8,
    /// Tx Power
    pub tx_power: TxPower,
    /// Maximum payload size in bytes that the driver will send or receive.
    ///
    /// This allows for a more efficient usage of the receiver queue and makes this driver
    /// compatible with nRF24L01+ modules when this size is 32 bytes or less
    pub maximum_payload_size: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            wait_for_ack_timeout: RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS,
            retransmit_delay: RETRANSMIT_DELAY,
            maximum_transmit_attempts: MAXIMUM_TRANSMIT_ATTEMPTS,
            enabled_pipes: ENABLED_PIPES,
            tx_power: TxPower::_0_DBM,
            maximum_payload_size: 252,
        }
    }
}
