use ch32_hal::exti::ExtiInput;
use ch32_hal::gpio::Output;

use embassy_time::{with_timeout, Duration, TimeoutError, Timer};

#[cfg(feature = "debug")]
use ch32_util::sdi_write::sdi_write;

use portable_atomic::Ordering;

use crate::led_state::LedState;
use crate::{LED_STATE, POWER_STATE};

const DEBOUNCE_MS: u64 = 20;
const FLASH_MS: u64 = 500;
const LONG_PRESS_MS: u64 = 2000;

pub async fn button_task(mut button: ExtiInput<'static>, mut pc817: Output<'static>) {
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
                    #[cfg(feature = "debug")]
                    sdi_write(b">> SHORT PRESS");
                    if !POWER_STATE.load(Ordering::Relaxed) {
                        pc817.set_high();
                        POWER_STATE.store(true, Ordering::Relaxed);
                        LED_STATE.store(LedState::On as u8, Ordering::Relaxed);
                    }
                }
                Err(TimeoutError) => {
                    #[cfg(feature = "debug")]
                    sdi_write(b">> FLASH");
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
                            #[cfg(feature = "debug")]
                            sdi_write(b">> LONG PRESS");
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
