/// PID (Packet ID) is an ID to identify duplicates of transmitted data, etc.
///
/// This ID is a 2-bit value, which is incremented by 1 in the order in which packets are sent, returning to 00 after 11.
/// By examining this value, it is possible to tell if a packet sent was retransmitted or if a packet was left out, etc.
#[derive(Default, PartialEq, Eq, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Pid(u8);
impl Pid {
    /// Creates a new instance of `Pid`
    ///
    /// Safety: value must be 0..=3
    pub(crate) fn new_unchecked(val: u8) -> Self {
        Self(val)
    }

    pub fn inner(&self) -> u8 {
        self.0
    }

    /// Move to the next PID
    pub fn go_next(&mut self) {
        if self.0 == 0b11 {
            self.0 = 0;
        } else {
            self.0 += 1;
        }
    }

    /// Check if the provided PID is the next PID
    pub fn is_next(&self, other: &Self) -> bool {
        if self.0 == 0b11 {
            other.0 == 0
        } else {
            self.0 + 1 == other.0
        }
    }
}
