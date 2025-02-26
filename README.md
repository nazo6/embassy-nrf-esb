# ESB protocol implementation for embassy-nrf

> [!NOTE]
>
> This library is a work in progress and is not yet ready for use.

## Example

- PRX side

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::config::HfclkSource;
use embassy_nrf::{bind_interrupts, config::Config, peripherals::RADIO};
use embassy_nrf_esb::{prx::PrxRadio, RadioConfig};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    RADIO => embassy_nrf_esb::InterruptHandler<RADIO>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    // Make sure to set the HFCLK source to ExternalXtal. Otherwise, the radio will not work.
    config.hfclk_source = HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    let mut prx = PrxRadio::<'_, _, 64>::new(p.RADIO, Irqs, RadioConfig::default()).unwrap();
    loop {
        let mut buf = [0; 64];
        let res = prx.recv(&mut buf, 0xFF).await;
        defmt::info!("{:?}, {:?}", buf, res);
    }
}
```

- PTX side

```rust
#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::config::HfclkSource;
use embassy_nrf::{bind_interrupts, config::Config, peripherals::RADIO};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(pub struct Irqs {
    RADIO => embassy_nrf_esb::InterruptHandler<RADIO>;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    // Make sure to set the HFCLK source to ExternalXtal. Otherwise, the radio will not work.
    config.hfclk_source = HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(config);

    let mut ptx = embassy_nrf_esb::ptx::PtxRadio::<'_, _, 64>::new(
        p.RADIO,
        Irqs,
        embassy_nrf_esb::RadioConfig::default(),
        embassy_nrf_esb::ptx::PtxConfig::default(),
    )
    loop {
        ptx
            .send(0, &[0, 1, 2, 3], false)
            .await
            .unwrap();
    }
}
```

- PTX side
