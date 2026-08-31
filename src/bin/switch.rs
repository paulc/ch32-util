#![no_std]
#![no_main]

use ch32_hal::adc::{Adc, SampleTime, Vref};
use ch32_hal::debug::SDIPrint;
use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::{Level, Output, Pull};

use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration, TimeoutError, Timer};

use ch32_util::chip_info::{clear_reset, decode_reset};
use ch32_util::iwdg::{Prescaler, Watchdog};
use ch32_util::sdi_println;

use portable_atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

#[repr(u8)]
enum LedState {
    Off = 0,
    On = 1,
    SlowFlash = 2,
    FastFlash = 3,
}

impl TryFrom<u8> for LedState {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0 => Ok(LedState::Off),
            1 => Ok(LedState::On),
            2 => Ok(LedState::SlowFlash),
            3 => Ok(LedState::FastFlash),
            other => Err(other),
        }
    }
}

static COUNTER: AtomicU32 = AtomicU32::new(0);
static POWER_STATE: AtomicBool = AtomicBool::new(false);
static LED_STATE: AtomicU8 = AtomicU8::new(0);

const DEBOUNCE_MS: u64 = 20;
const FLASH_MS: u64 = 500;
const LONG_PRESS_MS: u64 = 2000;
const WATCHDOG_MS: u32 = 4000;
const VREF_MV: u32 = 1228000; // 1.2V Ref * 1024 * 1000

#[embassy_executor::task]
async fn led_task(mut led: Output<'static>, mut wdt: Watchdog) {
    loop {
        match TryInto::<LedState>::try_into(LED_STATE.load(Ordering::Relaxed)).unwrap() {
            LedState::Off => {
                led.set_low();
                Timer::after_millis(200).await;
            }
            LedState::On => {
                led.set_high();
                Timer::after_millis(200).await;
            }
            LedState::SlowFlash => {
                led.toggle();
                Timer::after_millis(500).await;
            }
            LedState::FastFlash => {
                led.toggle();
                Timer::after_millis(100).await;
            }
        }
        // Feed IWDG
        wdt.feed();
    }
}

#[embassy_executor::task]
async fn button_task(mut button: ExtiInput<'static>, mut pc817: Output<'static>) {
    loop {
        button.wait_for_falling_edge().await;
        // Debounce
        Timer::after_millis(DEBOUNCE_MS).await;
        if button.is_low() {
            match with_timeout(
                Duration::from_millis(FLASH_MS),
                button.wait_for_rising_edge(),
            )
            .await
            {
                Ok(_) => {
                    sdi_println!(">> SHORT PRESS");
                    if !POWER_STATE.load(Ordering::Relaxed) {
                        pc817.set_high();
                        POWER_STATE.store(true, Ordering::Relaxed);
                        LED_STATE.store(LedState::On as u8, Ordering::Relaxed);
                    }
                }
                Err(TimeoutError) => {
                    sdi_println!(">> FLASH");
                    let prev = LED_STATE.swap(LedState::FastFlash as u8, Ordering::Relaxed);
                    match with_timeout(
                        Duration::from_millis(LONG_PRESS_MS),
                        button.wait_for_rising_edge(),
                    )
                    .await
                    {
                        Ok(_) => {
                            LED_STATE.store(prev, Ordering::Relaxed);
                        }
                        Err(TimeoutError) => {
                            sdi_println!(">> LONG PRESS");
                            if POWER_STATE.load(Ordering::Relaxed) {
                                pc817.set_low();
                                POWER_STATE.store(false, Ordering::Relaxed);
                                LED_STATE.store(LedState::Off as u8, Ordering::Relaxed);
                            } else {
                                LED_STATE.store(prev, Ordering::Relaxed);
                            }
                        }
                    }
                }
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

    let led = p.PA1;
    let button = p.PA2;
    let pc817 = p.PC4;

    // LED Task
    sdi_println!(">> Start led_task");
    let led = Output::new(led, Level::Low, Default::default());
    match led_task(led, wdt) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning led_task"),
    }

    let button = ExtiInput::new(button, p.EXTI2, Pull::Up);
    let pc817 = Output::new(pc817, Level::Low, Default::default());

    sdi_println!(">> Start button_task");
    match button_task(button, pc817) {
        Ok(t) => spawner.spawn(t),
        Err(_) => panic!("Error spawning button_task"),
    }

    loop {
        sdi_println!(">> COUNTER: {}", COUNTER.fetch_add(1, Ordering::Relaxed));
        Timer::after_millis(1000).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
