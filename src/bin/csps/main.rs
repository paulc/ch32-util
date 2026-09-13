#![no_std]
#![no_main]

use ch32_hal::adc::{Adc, SampleTime, Vref};
use ch32_hal::gpio::{Input, Level, Output, Pull};
use ch32_hal::i2c::I2c;
use ch32_hal::time::Hertz;
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Ticker};

use ch32_util::chip_info::clear_reset;
use ch32_util::iwdg::{Prescaler, Watchdog};

use portable_atomic::{AtomicBool, AtomicU16, AtomicU32, AtomicU8, Ordering};

mod led_state;
mod line_handler;
mod parse;
mod serial;
mod stack;

use led_state::LedState;
use serial::serial_write;

static POWER_STATE: AtomicBool = AtomicBool::new(false);
static LED_STATE: AtomicU8 = AtomicU8::new(0);
static UPTIME: AtomicU32 = AtomicU32::new(0);
static ON_TIMER: AtomicU32 = AtomicU32::new(0);
static OFF_TIMER: AtomicU32 = AtomicU32::new(0);
static POWER_OFF_DELAY: AtomicU32 = AtomicU32::new(5);
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
    // Paint stack at init
    stack::paint_stack();

    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    clear_reset();

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

    serial_write(b"-- [[ CH32V003 CSPS ]] --\r\n");
    stack::print_stack();

    // I2C
    let i2c = I2c::new_blocking(p.I2C1, scl, sda, Hertz::hz(100_000), Default::default());

    match line_handler::line_handler(i2c) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("line_handler"),
    }

    // Status task
    let button = Input::new(switch, Pull::Up);
    let mut enable = Output::new(enable, Level::Low, Default::default());
    let mut board_led = Output::new(board_led, Level::Low, Default::default());
    let mut _ext_led = Output::new(ext_led, Level::Low, Default::default());
    let ps_alarm = Input::new(ps_alarm, Pull::Down);
    let ps_ok = Input::new(ps_ok, Pull::Down);
    let mut adc = Adc::new(p.ADC1, Default::default());

    // Watchdog
    let mut wdt = match Watchdog::start(Prescaler::Div256, WATCHDOG_MS) {
        Ok(w) => w,
        Err(_) => panic!("watchdog"),
    };

    const STATUS_TICKER_MS: u32 = 50;

    let status_task = async {
        let mut count = 0_u32;
        let mut ticker = Ticker::every(Duration::from_millis(STATUS_TICKER_MS as u64));
        let mut button_ms = 0_u32;
        let mut off_warning = false;

        // Track uptime from ms timer ticks (avoids u64_div_rem)
        let mut last_ms = Instant::now().as_ticks() as u32; // TICK_HZ == 1000
        let mut frac_ms = 0_u32;
        let mut now_s = 0_u32;

        loop {
            // Get uptine in secs
            let ms = Instant::now().as_ticks() as u32;
            frac_ms += ms.wrapping_sub(last_ms);
            last_ms = ms;
            while frac_ms >= 1000 {
                frac_ms -= 1000;
                now_s = now_s.wrapping_add(1);
            }

            // Update status
            UPTIME.store(now_s, Ordering::Relaxed);
            PS_OK.store(ps_ok.is_high(), Ordering::Relaxed);
            PS_ALARM.store(ps_alarm.is_high(), Ordering::Relaxed);
            VREF.store(
                adc.convert(&mut Vref, SampleTime::CYCLES57),
                Ordering::Relaxed,
            );
            IMON.store(
                adc.convert(&mut imon, SampleTime::CYCLES57),
                Ordering::Relaxed,
            );

            // Check for off-warning
            off_warning = if OFF_TIMER.load(Ordering::Relaxed) > 0
                && now_s
                    >= OFF_TIMER.load(Ordering::Relaxed) - POWER_OFF_DELAY.load(Ordering::Relaxed)
            {
                // Only send warning once
                if !off_warning {
                    serial_write(b"!! POWERING DOWN !!\r\n");
                }
                true
            } else {
                false
            };

            // Check button status
            if button.is_low() {
                button_ms = button_ms.saturating_add(STATUS_TICKER_MS);
            } else {
                button_ms = 0;
            }

            // Check for short/long press
            match button_ms {
                100..=500 => {
                    // Short Press
                    if !POWER_STATE.load(Ordering::Relaxed) {
                        ON_TIMER.store(now_s, Ordering::Relaxed);
                    }
                    // If off-warning is active cancel
                    if off_warning {
                        OFF_TIMER.store(0, Ordering::Relaxed);
                    }
                }
                2000..=5000 => {
                    // Long Press
                    if POWER_STATE.load(Ordering::Relaxed) {
                        OFF_TIMER.store(
                            now_s + POWER_OFF_DELAY.load(Ordering::Relaxed),
                            Ordering::Relaxed,
                        );
                    }
                }
                _ => {}
            }

            // Check power on timer
            if ON_TIMER.load(Ordering::Relaxed) > 0 && now_s >= ON_TIMER.load(Ordering::Relaxed) {
                enable.set_high();
                POWER_STATE.store(true, Ordering::Relaxed);
                serial_write(b"!! POWER ON !!\r\n");
                ON_TIMER.store(0, Ordering::Relaxed);
            }

            // Check power off timer
            if OFF_TIMER.load(Ordering::Relaxed) > 0 && now_s >= OFF_TIMER.load(Ordering::Relaxed) {
                enable.set_low();
                POWER_STATE.store(false, Ordering::Relaxed);
                serial_write(b"!! POWER OFF !!\r\n");
                OFF_TIMER.store(0, Ordering::Relaxed);
            }

            // Update LED state
            if button_ms >= 500 && button_ms <= 2000 {
                LED_STATE.store(LedState::FastFlash as u8, Ordering::Relaxed);
            } else {
                LED_STATE.store(
                    if off_warning {
                        LedState::SlowFlash as u8
                    } else if POWER_STATE.load(Ordering::Relaxed) {
                        LedState::On as u8
                    } else {
                        LedState::Off as u8
                    },
                    Ordering::Relaxed,
                )
            }

            // Update LED
            match LedState::from(LED_STATE.load(Ordering::Relaxed)) {
                LedState::Off => board_led.set_low(),
                LedState::On => board_led.set_high(),
                LedState::SlowFlash => {
                    if count.is_multiple_of(8) {
                        board_led.toggle()
                    }
                }
                LedState::FastFlash => {
                    if count.is_multiple_of(2) {
                        board_led.toggle()
                    }
                }
                _ => {}
            }

            count = count.wrapping_add(1);
            wdt.feed();
            ticker.next().await;
        }
    };

    // let _ = join(status_task, button_task::button_task(button, enable)).await;

    status_task.await;

    loop {
        let _ = core::future::pending::<()>().await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
