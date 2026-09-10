#![no_std]
#![no_main]

#[cfg(feature = "debug")]
use ch32_hal::adc::{Adc, SampleTime, Vref};
use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Input, Level, Output, Pull};
use ch32_hal::i2c::I2c;
use ch32_hal::time::Hertz;
use ch32_hal::usart;

#[cfg(feature = "debug")]
use ch32_hal::debug::SDIPrint;

use embassy_executor::Spawner;
use embassy_futures::join::join3;
use embassy_time::Timer;

use ch32_util::chip_info::clear_reset;
#[cfg(feature = "debug")]
use ch32_util::chip_info::decode_reset;
use ch32_util::iwdg::{Prescaler, Watchdog};

use portable_atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};

mod button_task;
mod led_state;
mod led_task;
mod line_handler;
mod parse;
mod serial;

static POWER_STATE: AtomicBool = AtomicBool::new(false);
static LED_STATE: AtomicU8 = AtomicU8::new(0);
static VREF: AtomicU16 = AtomicU16::new(0);
static IMON: AtomicU16 = AtomicU16::new(0);
static PS_OK: AtomicBool = AtomicBool::new(false);
static PS_ALARM: AtomicBool = AtomicBool::new(false);

const WATCHDOG_MS: u32 = 4000;
// const VREF_MV: u32 = 1228000; // 1.2V Ref * 1024 * 1000

ch32_hal::bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    #[cfg(feature = "debug")]
    SDIPrint::enable();

    #[cfg(feature = "debug")]
    {
        ch32_util::stack_info::paint_stack();
        ch32_util::stack_info::print_stack_info(b"INIT");
    }

    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    #[cfg(feature = "debug")]
    decode_reset(ch32_hal::pac::RCC.rstsckr().read().0);

    clear_reset();

    let wdt = match Watchdog::start(Prescaler::Div256, WATCHDOG_MS) {
        Ok(w) => w,
        Err(_) => panic!("watchdog"),
    };

    // IO Pins
    let board_led = p.PC0;
    let sda = p.PC1;
    let scl = p.PC2;
    let enable = p.PC3;
    let switch = p.PC4;
    let ext_led = p.PC6;
    let ps_alarm = p.PC7;
    let mcu_tx = p.PD5;
    let mcu_rx = p.PD6;
    let mut imon = p.PA1;
    let ps_ok = p.PA2;

    // I2C
    let i2c = I2c::new_blocking(p.I2C1, scl, sda, Hertz::hz(100_000), Default::default());

    // Serial
    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (uart_tx, uart_rx) = match usart::Uart::new(
        p.USART1,
        mcu_rx,
        mcu_tx,
        Irqs,
        p.DMA1_CH4,
        p.DMA1_CH5,
        uart_config,
    ) {
        Ok(uart) => uart.split(),
        Err(_) => {
            panic!("uart");
        }
    };

    serial::UCASE.store(true, Ordering::Relaxed);

    // Spawn serial reader/writer tasks
    match serial::serial_read_task(uart_rx) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("serial_read"),
    }
    match serial::serial_write_task(uart_tx) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("serial_write"),
    }
    match line_handler::line_handler(i2c) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("line_handler"),
    }

    // LED + Button Task
    let board_led = Output::new(board_led, Level::Low, Default::default());
    let button = ExtiInput::new(switch, p.EXTI4, Pull::Up);
    let enable = Output::new(enable, Level::Low, Default::default());

    // Status task
    let _ext_led = Output::new(ext_led, Level::Low, Default::default());
    let ps_alarm = Input::new(ps_alarm, Pull::Down);
    let ps_ok = Input::new(ps_ok, Pull::Down);
    let mut adc = Adc::new(p.ADC1, Default::default());

    let status_task = async {
        loop {
            PS_OK.store(ps_ok.is_high(), Ordering::Relaxed);
            PS_ALARM.store(ps_alarm.is_high(), Ordering::Relaxed);
            VREF.store(
                adc.convert(&mut Vref, SampleTime::CYCLES73),
                Ordering::Relaxed,
            );
            IMON.store(
                adc.convert(&mut imon, SampleTime::CYCLES73),
                Ordering::Relaxed,
            );
            Timer::after_millis(200).await;
        }
    };

    let _ = join3(
        status_task,
        button_task::button_task(button, enable),
        led_task::led_task(board_led, wdt),
    )
    .await;

    loop {
        let _ = core::future::pending::<()>().await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
