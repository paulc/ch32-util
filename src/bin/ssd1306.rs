#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::debug::SDIPrint;
use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Level, Output, Pull};
use ch32_hal::i2c::I2c;
use ch32_hal::mode::Async;
use ch32_hal::peripherals::I2C1;
use ch32_hal::time::Hertz;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker, Timer};

use static_cell::StaticCell;

use ch32_util::chip_info::{clear_reset, decode_reset};
use ch32_util::iwdg::{Prescaler, Watchdog};
use ch32_util::sdi_println;
use ch32_util::ssd1306_async::{Rotation, Size, Ssd1306};

use portable_atomic::{AtomicU32, Ordering};

bind_interrupts!(struct Irqs {
    I2C1_EV => ch32_hal::i2c::EventInterruptHandler<ch32_hal::peripherals::I2C1>;
    I2C1_ER => ch32_hal::i2c::ErrorInterruptHandler<ch32_hal::peripherals::I2C1>;
});

static COUNTER: AtomicU32 = AtomicU32::new(0);
static I2C_BUS: StaticCell<Mutex<NoopRawMutex, I2c<'static, I2C1, Async>>> = StaticCell::new();

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>, mut wdt: Watchdog, interval_ms: u64) {
    loop {
        led.set_high();
        Timer::after_millis(interval_ms).await;
        led.set_low();
        Timer::after_millis(interval_ms).await;
        // Feed IWDG
        wdt.feed();
    }
}

#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>) {
    loop {
        button.wait_for_rising_edge().await;
        // Debounce
        Timer::after_millis(20).await;
        if button.is_high() {
            sdi_println!(">> BUTTON");
            COUNTER.store(0, Ordering::Relaxed);
        }
    }
}

#[embassy_executor::task]
async fn display_task(
    i2c: I2cDevice<'static, NoopRawMutex, I2c<'static, I2C1, Async>>,
    interval: u64,
) {
    sdi_println!("Init Display");

    let mut display = Ssd1306::new(i2c, 0x3C, Size::S128x64);
    display.init(Rotation::Deg0).await.ok();
    display.clear().await.ok();
    let mut buf = heapless::String::<16>::new();
    let mut ticker = Ticker::every(Duration::from_millis(interval));
    loop {
        // sdi_println!(">> Display: {}", count);
        ticker.next().await;
        for line in 0..8 {
            buf.clear();
            let _ = ufmt::uwrite!(
                &mut buf,
                "LINE: {} <{}>",
                line,
                COUNTER.load(Ordering::Relaxed)
            );
            if display.write_line(line, buf.as_str()).await.is_err() {
                sdi_println!("!! ERROR: write_line");
            }
        }
        COUNTER.add(1, Ordering::Relaxed);
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    SDIPrint::enable();
    ch32_util::stack_info::paint_stack();
    ch32_util::stack_info::print_stack_info(b"INIT");

    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    sdi_println!(">> INIT");

    decode_reset(ch32_hal::pac::RCC.rstsckr().read().0);
    clear_reset();

    let wdt = match Watchdog::start(Prescaler::Div256, 4000) {
        Ok(w) => w,
        Err(_) => panic!("bad watchdog period"),
    };

    // LED Task
    sdi_println!("Start led_task");
    let led = Output::new(p.PC3, Level::Low, Default::default());
    match led_task(led, wdt, 500) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning led_task"),
    }

    let button = ExtiInput::new(p.PC0, p.EXTI0, Pull::Down);

    sdi_println!("Start button_task");
    match button_task(button) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning led_task"),
    }

    let scl = p.PC2;
    let sda = p.PC1;

    let mut i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::hz(400_000),
        Default::default(),
    );

    #[cfg(feature = "debug")]
    {
        sdi_println!("Scan I2C bus: START");
        for addr in 1..=127 {
            if i2c.blocking_write(addr, &[0]).is_ok() {
                sdi_println!(">> Found I2C device at address: 0x{:02x}", addr);
            }
        }
        sdi_println!("Scan I2C bus: DONE");
    }

    // Create shared I2C Bus
    let i2c_bus = I2C_BUS.init(Mutex::new(i2c));
    let ssd1306_device = I2cDevice::new(i2c_bus);

    sdi_println!("Start led_task");
    match display_task(ssd1306_device, 100) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning display_task"),
    }

    loop {
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
