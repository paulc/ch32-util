use ch32_hal::gpio::Output;

use embassy_time::Timer;
use portable_atomic::Ordering;

use ch32_util::iwdg::Watchdog;

use crate::led_state::LedState;
use crate::LED_STATE;

#[embassy_executor::task]
pub async fn led_task(mut led: Output<'static>, mut wdt: Watchdog) {
    loop {
        match Into::<LedState>::into(LED_STATE.load(Ordering::Relaxed)) {
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
            _ => {}
        }
        // Feed IWDG
        wdt.feed();
    }
}
