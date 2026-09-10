use ch32_hal::gpio::Output;

use embassy_time::Timer;
use portable_atomic::Ordering;

use ch32_util::iwdg::Watchdog;

use crate::led_state::LedState;
use crate::LED_STATE;

pub async fn led_task(mut led: Output<'static>, mut wdt: Watchdog) -> ! {
    loop {
        let ms = match LedState::from(LED_STATE.load(Ordering::Relaxed)) {
            LedState::Off => {
                led.set_low();
                200
            }
            LedState::On => {
                led.set_high();
                200
            }
            LedState::SlowFlash => {
                led.toggle();
                500
            }
            LedState::FastFlash => {
                led.toggle();
                100
            }
            LedState::Unknown => 200,
        };
        wdt.feed();
        Timer::after_millis(ms).await;
    }
}
