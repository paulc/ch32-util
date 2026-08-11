#![no_std]
#![no_main]

use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_time::Timer;

use ch32_util::chip_info::{clear_reset};
use ch32_util::ufmt::UartFmt;

use ufmt::uwriteln;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    // UART
    let uart_config = usart::Config::default(); // 115200,N,8,1
    let uart_tx = match usart::UartTx::new_blocking(p.USART1, p.PD0, uart_config) {
        Ok(tx) => tx,
        Err(_) => {
            panic!("UART_TX");
        }
    };

    let reset = ch32_hal::pac::RCC.rstsckr().read().0;
    clear_reset();

    let mut uart = UartFmt::new(uart_tx);
    let _ = uwriteln!(&mut uart, "\n>> CH32V003 <<\n");
    let _ = uwriteln!(&mut uart, "RESET: 0x{:x}", reset);

    // LED
    let mut led = Output::new(p.PC3, Level::Low, Default::default());
    let mut i: u32 = 0;
    loop {
        let _ = uwriteln!(&mut uart, ">> count = {}", i);
        led.toggle();
        i += 1;
        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
