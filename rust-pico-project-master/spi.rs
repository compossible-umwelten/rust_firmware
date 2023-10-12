//! # SPI Example
//!
//! This application demonstrates how to use the SPI Driver to talk to a remote
//! SPI device.
//!
//!
//! It may need to be adapted to your particular board layout and/or pin
//! assignment.
//!
//! See the `Cargo.toml` file for Copyright and license details.

#![no_std]
#![no_main]
extern crate panic_halt;
extern crate embedded_hal;
extern crate rp2040_hal;
extern crate fugit;
extern crate cortex_m;
extern crate log;
// Ensure we halt the program on panic (if we don't mention this crate it won't
// be linked)
//use panic_halt as _;

// Alias for our HAL crate
use rp2040_hal as hal;

// Some traits we need
use cortex_m::prelude::*;
use fugit::RateExtU32;
use rp2040_hal::clocks::Clock;
// A shorter alias for the Peripheral Access Crate, which provides low-level
// register access
use hal::pac;
//use embedded_hal::digital::v2::OutputPin;
//use cortex_m::delay::Delay

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

/// External high-speed crystal on the Raspberry Pi Pico board is 12 MHz. Adjust
/// if your board has a different frequency
const XTAL_FREQ_HZ: u32 = 12_000_000u32;
/// Entry point to our bare-metal application.
///
/// The `#[rp2040_hal::entry]` macro ensures the Cortex-M start-up code calls this function
/// as soon as all global variables and the spinlock are initialised.
///
/// The function configures the RP2040 peripherals, then performs some example
/// SPI transactions, then goes to sleep.
#[rp2040_hal::entry]
fn main() -> ! {
    // Grab our singleton objects
    //defmt::println!("Connected!");
    let mut pac = pac::Peripherals::take().unwrap();

    // Set up the watchdog driver - needed by the clock setup code
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    // Configure the clocks
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    // The single-cycle I/O block controls our GPIO pins
    let sio = hal::Sio::new(pac.SIO);

    // Set the pins to their default state
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let core = pac::CorePeripherals::take().unwrap();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // These are implicitly used by the spi driver if they are in the correct mode
    let _spi_mosi = pins.gpio7.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_miso = pins.gpio4.into_mode::<hal::gpio::FunctionSpi>();
    let _spi_sclk = pins.gpio6.into_mode::<hal::gpio::FunctionSpi>();
    //let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (spi_mosi, spi_miso, spi_sclk));
    let spi = hal::spi::Spi::<_, _, 8>::new(pac.SPI0);

    // Exchange the uninitialised SPI driver for an initialised one
    let mut spi = spi.init(
        &mut pac.RESETS,
        clocks.peripheral_clock.freq(),
        16.MHz(),
        &embedded_hal::spi::MODE_0,
    );

    // Reset the BOS1901
    let config_register_address = 0x05; // Address of the CONFIG register
    let reset_value = 0x1 << 5;  // Set the RST bit
    spi.write(&[config_register_address, reset_value]).unwrap();

    // Enable the Output
    let output_enable_value = 0x1 << 4;  // Set the OE bit
    spi.write(&[config_register_address, output_enable_value]).unwrap();

    let sup_rise_register_address = 0x7; // Address of the SUP_RISE register
    let sense_bit_value = 0x0;  // Value to set the SENSE bit to 0
    spi.write(&[sup_rise_register_address, sense_bit_value]).unwrap();

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

    // Number of times to alternate between high and low values
    let repetitions = 10;

    // Send the square wave to the FIFO
    for _ in 0..repetitions {
        // Send high value
        spi.write(&[reference_register_address, high_value_high, high_value_low]).unwrap();
        // Delay to maintain the high value for a certain duration (adjust as needed)
        delay.delay_ms(500);

        // Send low value
        spi.write(&[reference_register_address, low_value_high, low_value_low]).unwrap();
        // Delay to maintain the low value for a certain duration (adjust as needed)
        delay.delay_ms(500);
    }

    // After sending the square wave, send the stabilization value
    let stabilization_value: u16 = 0x0FFF; // Small negative voltage
    let stabilization_value_high = (stabilization_value >> 8) as u8;
    let stabilization_value_low = (stabilization_value & 0xFF) as u8;
    spi.write(&[reference_register_address, stabilization_value_high, stabilization_value_low]).unwrap();

    loop {
        cortex_m::asm::wfi();
    }
}

// End of file