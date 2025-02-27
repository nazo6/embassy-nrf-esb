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
