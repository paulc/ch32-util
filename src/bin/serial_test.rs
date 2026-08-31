#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::task]
async fn idle_task() {
    loop {
        Timer::after_millis(1000).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let config = ch32_hal::Config::default();
    let p = ch32_hal::init(config);

    let mut led = Output::new(p.PC3, Level::Low, Default::default());

    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (mut uart_tx, mut uart_rx) = match usart::Uart::new(
        p.USART1,
        p.PC1, // RX
        p.PC0, // TX
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

    spawner.spawn(idle_task().unwrap());

    for _ in 0..5 {
        let _ = uart_tx.write(b"ABCDEFGHIJKLMNOPQRSTUVWXYZ\r\n").await;
    }

    let mut buf = [0u8; 64];
    loop {
        match uart_rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                led.set_high();
                let _ = uart_tx.write(b">> ").await;
                let _ = uart_tx.write(&buf[..n]).await;
                let _ = uart_tx.write(b"\r\n").await;
                led.set_low();
            }
            Err(e) => {
                let _ = match e {
                    usart::Error::Overrun => uart_tx.write(b"-- ERR overrun\r\n").await,
                    usart::Error::Framing => uart_tx.write(b"-- ERR framing\r\n").await,
                    usart::Error::Noise => uart_tx.write(b"-- ERR noise\r\n").await,
                    _ => uart_tx.write(b"-- ERR other\r\n").await,
                };
            }
        }
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
