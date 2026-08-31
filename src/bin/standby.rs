#![no_std]
#![no_main]

use ch32_hal::debug::SDIPrint;
use ch32_hal::gpio::{Level, Output};

use embassy_executor::Spawner;
use embassy_time::Timer;

use ch32_util::awu;
use ch32_util::chip_info::{clear_reset, decode_reset};

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    SDIPrint::enable();

    let config = ch32_hal::Config::default();
    let p = ch32_hal::init(config);

    ch32_util::sdi_write::sdi_write(b">> INIT\n");

    // Delay to allow programming after reboot
    Timer::after_millis(1000).await;

    decode_reset(ch32_hal::pac::RCC.rstsckr().read().0);
    clear_reset();

    // Configure AWU
    if awu::pwr_enable().is_err() {
        panic!("PWR_ENABLE");
    }
    awu::lsi_enable().ok();
    awu::awu_configure(awu::Prescaler::Div61440, 5_000).ok();
    awu::awu_enable();

    let mut led = Output::new(p.PC3, Level::Low, Default::default());
    let mut count = 1_u64;
    loop {
        flash(&mut led, count).await;
        awu::standby();
        Timer::after_millis(250).await;
        count += 1;
    }
}

async fn flash(led: &mut Output<'_>, count: u64) {
    for _ in 0..count {
        led.set_high();
        Timer::after_millis(100).await;
        led.set_low();
        Timer::after_millis(100).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
