#![no_std]
#![no_main]
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::prelude::_embedded_hal_blocking_spi_Write;
use fugit::RateExtU32;
use panic_probe as _;
use rp2040_hal as hal;

use bos1901::reg;

use hal::{
    clocks::Clock,
    gpio::{FunctionSpi, Pins},
};

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
// for some reason this bootloader is not working for arduino rp2040 connect.
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut pac = hal::pac::Peripherals::take().unwrap();
    let core = hal::pac::CorePeripherals::take().unwrap();
    let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        12_000_000u32,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let sio = hal::Sio::new(pac.SIO);
    let pins = Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut gpio6 = pins.gpio6.into_push_pull_output();
    for _ in 0..10 {
        gpio6.set_high().unwrap();
        delay.delay_ms(50);
        gpio6.set_low().unwrap();
        delay.delay_ms(50);
    }
    // cs is on gpio5 for now
    let mut gpio5 = pins.gpio5.into_push_pull_output();
    gpio5.set_low().unwrap();
    let mosi = pins.gpio7.into_function::<FunctionSpi>();
    let miso = pins.gpio4.into_function::<FunctionSpi>();
    let sclk = gpio6.into_function::<FunctionSpi>();
    // let spi = hal::spi::Spi::new(pac.SPI0, (mosi, miso, sclk));
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16u32.MHz(),
        embedded_hal::spi::MODE_0,
    );
    let mut config = 0x0;
    config |= 0x1 << 5; // Set the RST bit
    config |= 0x1 << 4; // Set the OE bit
    spi.write(&[reg::CONFIG, config]).unwrap();

    let sup_rise = 0x0; // Value to set the SENSE bit to 0
    spi.write(&[reg::SUP_RISE, sup_rise]).unwrap();

    loop {
        spi.write(&[reg::REFERENCE, 0x0f, 0xff]).unwrap();
        delay.delay_ms(100);
        spi.write(&[reg::REFERENCE, 0x00, 0x00]).unwrap();
        delay.delay_ms(100);
    }
    // loop {
    //     spi.write(b"hello").unwrap();
    // }
    // loop {
    //     cortex_m::asm::wfi();
    // }
}

mod bos1901;
