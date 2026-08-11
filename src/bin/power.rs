#![no_std]
#![no_main]

use ch32_hal::debug::SDIPrint;
use ch32_hal::gpio::{Level, Output};

use embassy_executor::Spawner;
use embassy_time::Timer;

use ch32_util::awu;
use ch32_util::chip_info::{clear_reset, decode_reset};
use ch32_util::sdi_write::FormatHex;
use ch32_util::sdi_writeln;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    // SDIPrint::enable();
    ch32_util::stack_info::paint_stack();
    ch32_util::stack_info::print_stack_info(b"INIT");

    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    ch32_util::sdi_write::sdi_write(b">> INIT\n");

    // Delay to allow programming after reboot
    Timer::after_millis(1000).await;

    decode_reset(ch32_hal::pac::RCC.rstsckr().read().0);
    clear_reset();

    // Enable LSI for AWU
    ch32_hal::pac::RCC.rstsckr().modify(|w| w.set_lsion(true));
    while !ch32_hal::pac::RCC.rstsckr().read().lsirdy() {}

    // Configure AWU
    if awu::pwr_enable().is_err() {
        sdi_writeln!(b"ERROR: PWR Enable");
        panic!("PWR_ENABLE");
    }
    awu::lsi_enable().ok();
    awu::awu_configure(awu::Prescaler::Div61440, 5_000).ok();
    awu::awu_enable();

    let r = awu::read_regs();
    sdi_writeln!(b"AWUCSR", &r.awucsr.fmt_hex());
    sdi_writeln!(b"AWUWR ", &r.awuwr.fmt_hex());
    sdi_writeln!(b"AWUPSC", &r.awupsc.fmt_hex());
    sdi_writeln!(b"RSTSCK", &r.rcc_rstsckr.fmt_hex());
    sdi_writeln!(b"EVENR ", &r.exti_evenr.fmt_hex());
    sdi_writeln!(b"SCTLR ", &r.pfic_sctlr.fmt_hex());

    let mut led = Output::new(p.PC3, Level::Low, Default::default());
    let mut count = 1_usize;
    loop {
        for _ in 0..10 {
            led.set_high();
            Timer::after_millis(50).await;
            led.set_low();
            Timer::after_millis(50).await;
        }
        ch32_util::sdi_write::sdi_write(b"SLEEP\n");
        awu::standby();
        // Wait for standby
        Timer::after_millis(100).await;
        ch32_util::sdi_write::sdi_write(b"WAKE\n");
        for _ in 0..count {
            led.set_high();
            Timer::after_millis(250).await;
            led.set_low();
            Timer::after_millis(250).await;
        }
        Timer::after_millis(1000).await;
        count += 1;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
