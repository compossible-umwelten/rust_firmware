#![no_std]
#![no_main]
use defmt::*;
use defmt_rtt as _;
use embedded_hal::{digital::v2::OutputPin, prelude::_embedded_hal_blocking_spi_Write};
use fugit::RateExtU32;
use panic_probe as _;
use rp2040_hal as hal;

use hal::clocks::Clock;

#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;
// for some reason this bootloader is not working for arduino rp2040 connect.
// pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Program start");
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

    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    // cs is on gpio5 for now
    let mut gpio5 = pins.gpio5.into_push_pull_output();
    gpio5.set_low().unwrap();
    let mut gpio6 = pins.gpio6.into_push_pull_output();
    gpio6.set_high().unwrap();
    delay.delay_ms(500);
    let mosi = pins.gpio7.into_function::<hal::gpio::FunctionSpi>();
    let miso = pins.gpio4.into_function::<hal::gpio::FunctionSpi>();
    let sclk = gpio6.into_function::<hal::gpio::FunctionSpi>();
    // let spi = hal::spi::Spi::new(pac.SPI0, (mosi, miso, sclk));
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));
    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16u32.MHz(),
        embedded_hal::spi::MODE_0,
    );

    // Reset the BOS1901
    let config_register_address = 0x05; // Address of the CONFIG register
    let reset_value = 0x1 << 5; // Set the RST bit
    spi.write(&[config_register_address, reset_value]).unwrap();

    // Enable the Output
    let output_enable_value = 0x1 << 4; // Set the OE bit
    spi.write(&[config_register_address, output_enable_value])
        .unwrap();

    let sup_rise_register_address = 0x7; // Address of the SUP_RISE register
    let sense_bit_value = 0x0; // Value to set the SENSE bit to 0
    spi.write(&[sup_rise_register_address, sense_bit_value])
        .unwrap();

    // Example: Sending a sample to the BOS1901's FIFO
    let reference_register_address = 0x00; // Based on the document
                                           // Define the high and low values for the square wave
    let high_value: u16 = 0xFFF;
    let low_value: u16 = 0x000;

    // Split the values into two u8 values for SPI transmission
    let high_value_high = (high_value >> 8) as u8;
    let high_value_low = (high_value & 0xFF) as u8;
    let low_value_high = (low_value >> 8) as u8;
    let low_value_low = (low_value & 0xFF) as u8;

    delay.delay_ms(500);

    let hello_slave = b"hello slave!";
    spi.write(hello_slave).unwrap();

    loop {
        // Send high value
        spi.write(&[reference_register_address, high_value_high, high_value_low])
            .unwrap();
        // Delay to maintain the high value for a certain duration (adjust as needed)
        delay.delay_ms(500);

        // Send low value
        spi.write(&[reference_register_address, low_value_high, low_value_low])
            .unwrap();
        // Delay to maintain the low value for a certain duration (adjust as needed)
        delay.delay_ms(500);
    }
}

struct Critical;
critical_section::set_impl!(Critical);
unsafe impl critical_section::Impl for Critical {
    unsafe fn acquire() -> critical_section::RawRestoreState {}
    unsafe fn release(_restore_state: critical_section::RawRestoreState) {}
}
