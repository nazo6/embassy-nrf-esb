/// Addresses used for communication.
///
/// ESB uses up to eight pipes to address communication, each pipe has an unique address which is
/// composed by the base address and the prefix. Pipe 0 has an unique base and prefix, while the
/// other pipes share a base address but have different prefixes.
///
/// Default values:
///
/// | Field      | Default Value            |
/// | :---       | :---                     |
/// | base0      | [0xE7, 0xE7, 0xE7, 0xE7] |
/// | base1      | [0xC2, 0xC2, 0xC2, 0xC2] |
/// | prefixes0  | [0xE7, 0xC2, 0xC3, 0xC4] |
/// | prefixes1  | [0xC5, 0xC6, 0xC7, 0xC8] |
/// | rf_channel | 2                        |
///
pub struct Addresses {
    /// Base address for pipe 0
    pub(crate) base0: [u8; 4],
    /// Base address for pipe 1-7
    pub(crate) base1: [u8; 4],
    /// Prefixes for pipes 0-3, in order
    pub(crate) prefixes0: [u8; 4],
    /// `prefixes1` - Prefixes for pipes 4-7, in order
    pub(crate) prefixes1: [u8; 4],
    /// Channel to be used by the radio hardware (must be between 0 and 100)
    pub(crate) rf_channel: u8,
}

pub const ADDR_LENGTH: u8 = 4 + 1;

impl Addresses {
    /// Creates a new instance of `Addresses`
    ///
    /// * `base0` - Base address for pipe 0.
    /// * `base1` - Base address for pipe 1-7.
    /// * `prefixes0` - Prefixes for pipes 0-3, in order.
    /// * `prefixes1` - Prefixes for pipes 4-7, in order.
    /// * `rf_channel` - Channel to be used by the radio hardware (must be between 0 and 100).
    pub fn new(
        base0: [u8; 4],
        base1: [u8; 4],
        prefixes0: [u8; 4],
        prefixes1: [u8; 4],
        rf_channel: u8,
    ) -> Result<Self, &'static str> {
        if rf_channel > 100 {
            return Err("invalid rf_channel");
        }
        Ok(Self {
            base0,
            base1,
            prefixes0,
            prefixes1,
            rf_channel,
        })
    }
}

impl Default for Addresses {
    fn default() -> Self {
        Self {
            base0: [0xE7, 0xE7, 0xE7, 0xE7],
            base1: [0xC2, 0xC2, 0xC2, 0xC2],
            prefixes0: [0xE7, 0xC2, 0xC3, 0xC4],
            prefixes1: [0xC5, 0xC6, 0xC7, 0xC8],
            rf_channel: 2,
        }
    }
}

#[inline]
pub fn address_conversion(value: u32) -> u32 {
    value.reverse_bits()
}
