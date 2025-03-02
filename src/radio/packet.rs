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

use crate::Error;

use super::pid::Pid;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Packet<const N: usize>(pub(crate) [u8; N]);

impl<const N: usize> Packet<N> {
    /// Creates new packet which is empty. This is used for receiving data.
    pub fn new_empty() -> Self {
        const {
            assert!(N >= 3);
        }
        Self([0; N])
    }

    fn payload_length(&self) -> u8 {
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

    pub(crate) fn set_ack(&mut self, ack: bool) {
        if ack {
            self.0[1] |= 1;
        } else {
            self.0[1] &= !1;
        }
    }

    pub(crate) fn set_pid(&mut self, pid: Pid) {
        self.0[1] &= 0b1;
        self.0[1] |= pid.inner() << 1;
    }

    pub(crate) fn set_payload(&mut self, payload: &[u8]) -> Result<(), Error> {
        if payload.len() > N - 2 {
            Err(Error::BufferTooLong)
        } else {
            self.0[0] = payload.len() as u8;
            self.0[2..payload.len() + 2].copy_from_slice(payload);
            Ok(())
        }
    }
}
