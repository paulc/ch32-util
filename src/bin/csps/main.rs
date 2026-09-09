#![no_std]
#![no_main]

use ch32_hal::adc::{Adc, SampleTime, Vref};
use ch32_hal::debug::SDIPrint;
use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Level, Output, Pull};
use ch32_hal::i2c::I2c;
use ch32_hal::mode::Async;
use ch32_hal::peripherals::I2C1;
use ch32_hal::time::Hertz;
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

use ch32_util::chip_info::{clear_reset, decode_reset};
use ch32_util::iwdg::{Prescaler, Watchdog};
use ch32_util::sdi_println;

use core::cell::RefCell;
use portable_atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

mod button_task;
mod led_state;
mod led_task;
mod serial;

static COUNTER: AtomicU32 = AtomicU32::new(0);
static POWER_STATE: AtomicBool = AtomicBool::new(false);
static LED_STATE: AtomicU8 = AtomicU8::new(0);

static I2C: Mutex<CriticalSectionRawMutex, RefCell<Option<I2c<'static, I2C1, Async>>>> =
    Mutex::new(RefCell::new(None));

const WATCHDOG_MS: u32 = 4000;
const VREF_MV: u32 = 1228000; // 1.2V Ref * 1024 * 1000

ch32_hal::bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
    I2C1_EV => ch32_hal::i2c::EventInterruptHandler<ch32_hal::peripherals::I2C1>;
    I2C1_ER => ch32_hal::i2c::ErrorInterruptHandler<ch32_hal::peripherals::I2C1>;
});

#[embassy_executor::task]
async fn line_handler() {
    loop {
        let line = serial::LINE_CHANNEL.receive().await;
        let line = line.trim_ascii();
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next() {
                Some("STATUS") => {
                    serial_println!(">> COUNTER: {}", COUNTER.load(Ordering::Relaxed));
                    serial_println!(">> POWER_STATE: {}", POWER_STATE.load(Ordering::Relaxed));
                    serial_println!(">> LED_STATE: {}", LED_STATE.load(Ordering::Relaxed));
                }
                Some("I2C") => match it.next() {
                    Some("SCAN") => {
                        serial_println!(">> Scan I2C bus: START");
                        let guard = I2C.lock().await;
                        for addr in 1..=127 {
                            if let Some(i2c) = guard.borrow_mut().as_mut() {
                                let mut buf = [0u8; 1];
                                if i2c.blocking_read(addr, &mut buf).is_ok() {
                                    serial_println!(
                                        ">>>> Found I2C device at address: 0x{:02x}",
                                        addr
                                    );
                                    // ALlow serial_write buffer to clear
                                    Timer::after_millis(10).await;
                                }
                            }
                        }
                        serial_println!(">> Scan I2C bus: DONE");
                    }
                    _ => {}
                },
                Some("ECHO") => {
                    match it.next() {
                        Some("ON") => serial::ECHO.store(true, Ordering::Relaxed),
                        Some("OFF") => serial::ECHO.store(false, Ordering::Relaxed),
                        _ => {}
                    }
                    serial_println!(
                        ">> ECHO {}",
                        if serial::ECHO.load(Ordering::Relaxed) {
                            "ON"
                        } else {
                            "OFF"
                        }
                    );
                }
                Some(s) => serial_println!("!! ERROR: <{}> [{}]", s, s.len()),
                None => {}
            }
            if serial::ECHO.load(Ordering::Relaxed) {
                serial_print!("## ");
            }
        }
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

    let mut adc = Adc::new(p.ADC1, Default::default());
    let vref = adc.convert(&mut Vref, SampleTime::CYCLES73);
    sdi_println!(">> ADC Vref: {} -> Vdd: {}mV", vref, VREF_MV / vref as u32);

    let wdt = match Watchdog::start(Prescaler::Div256, WATCHDOG_MS) {
        Ok(w) => w,
        Err(_) => panic!("bad watchdog period"),
    };

    let board_led = p.PC0;
    let sda = p.PC1;
    let scl = p.PC2;
    let enable = p.PC3;
    let switch = p.PC4;
    let ext_led = p.PC6;
    let psalarm = p.PC7;
    let mcu_tx = p.PD5;
    let mcu_rx = p.PD6;
    let imon = p.PA1;
    let psok = p.PA2;

    // I2C

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
            panic!("UART_TX");
        }
    };

    serial::UCASE.store(true, Ordering::Relaxed);

    // Spawn serial reader/writer tasks
    spawner.spawn(serial::serial_read(uart_rx).unwrap());
    spawner.spawn(serial::serial_write(uart_tx).unwrap());
    spawner.spawn(line_handler().unwrap());

    // I2C
    let i2c = I2c::new(
        p.I2C1,
        scl,
        sda,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz::hz(100_000),
        Default::default(),
    );

    {
        // Move I2C to static
        let guard = I2C.lock().await;
        *guard.borrow_mut() = Some(i2c);
    }

    // LED Task
    sdi_println!(">> Start led_task");
    let led = Output::new(board_led, Level::Low, Default::default());
    match led_task::led_task(led, wdt) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning led_task"),
    }

    let button = ExtiInput::new(switch, p.EXTI4, Pull::Up);
    let enable = Output::new(enable, Level::Low, Default::default());

    sdi_println!(">> Start button_task");
    match button_task::button_task(button, enable) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning button_task"),
    }

    loop {
        // sdi_println!(">> COUNTER: {}", COUNTER.fetch_add(1, Ordering::Relaxed));
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
