#![no_std]
#![no_main]

#[rtic::app(
    device = arduino_nano_connect::hal::pac,
    dispatchers = [TIMER_IRQ_1]
)]
mod app {
    use arduino_nano_connect::hal;
    use hal::{
        clocks, gpio,
        gpio::{
            bank0::{Gpio2, Gpio3, Gpio6},
            FunctionSio, SioOutput,
        },
        pac,
        sio::Sio,
        watchdog::Watchdog,
        I2C,
    };
    use arduino_nano_connect::XOSC_CRYSTAL_FREQ;
    use rp2040_monotonic::{
        fugit::Duration,
        // fugit::RateExtU32, // For .kHz() conversion funcs
        Rp2040Monotonic,
    };

    use core::mem::MaybeUninit;
    use embedded_hal::digital::v2::ToggleableOutputPin;

    use panic_probe as _;

    const MONO_NUM: u32 = 1;
    const MONO_DENOM: u32 = 1000000;
    const ONE_SEC_TICKS: u64 = 1000000;

    type I2CBus = I2C<
        pac::I2C1,
        (
            gpio::Pin<Gpio2, gpio::FunctionI2C, gpio::PullDown>,
            gpio::Pin<Gpio3, gpio::FunctionI2C, gpio::PullDown>,
        ),
    >;

    #[monotonic(binds = TIMER_IRQ_0, default = true)]
    type Rp2040Mono = Rp2040Monotonic;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: gpio::Pin<Gpio6, FunctionSio<SioOutput>, gpio::PullDown>,
        // i2c: &'static mut I2CBus,
    }

    #[init(local=[
        // Task local initialized resources are static
        // Here we use MaybeUninit to allow for initialization in init()
        // This enables its usage in driver initialization
        i2c_ctx: MaybeUninit<I2CBus> = MaybeUninit::uninit()
    ])]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // Configure the clocks, watchdog - The default is to generate a 125 MHz system clock
        let mut watchdog = Watchdog::new(ctx.device.WATCHDOG);
        let clocks = clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut ctx.device.RESETS,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        // Init LED pin
        let sio = Sio::new(ctx.device.SIO);
        let gpioa = arduino_nano_connect::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut ctx.device.RESETS,
        );
        // led.set_low().unwrap();

        // Init I2C pins
        // let miso = gpioa.cipo.into_function::<gpio::FunctionSpi>();
        // let mosi = gpioa.copi.into_function::<gpio::FunctionSpi>();
        // let sclk = gpioa.sck0.into_function::<gpio::FunctionSpi>();
        // let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi, miso, sclk));
        let led = gpioa.sck0.into_push_pull_output();
        let mono = Rp2040Mono::new(ctx.device.TIMER);

        // Spawn heartbeat task
        heartbeat::spawn().unwrap();

        // Return resources and timer
        (
            Shared {},
            // Local { led, i2c: i2c_tmp },
            Local { led },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led])]
    fn heartbeat(ctx: heartbeat::Context) {
        // Flicker the built-in LED
        _ = ctx.local.led.toggle();

        // Congrats, you can use your i2c and have access to it here,
        // now to do something with it!

        // Re-spawn this task after 1 second
        let one_second = Duration::<u64, MONO_NUM, MONO_DENOM>::from_ticks(ONE_SEC_TICKS);
        heartbeat::spawn_after(one_second).unwrap();
    }
}
