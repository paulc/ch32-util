#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_time::Timer;

use core::cell::RefCell;

use ch32_util::chip_info::clear_reset;
use ch32_util::ufmt::UartFmt;

pub static UART_TX: Mutex<RefCell<Option<UartFmt<ch32_hal::mode::Async>>>> =
    Mutex::new(RefCell::new(None));

// Helpers to print to formatted writer mutex
#[macro_export]
macro_rules! serial_println {
    ($($arg:tt)*) => {
        ::critical_section::with(|cs| {
            if let Some(uart) = $crate::UART_TX.borrow_ref_mut(cs).as_mut() {
                let _ = ::ufmt::uwriteln!(uart, $($arg)*);
            }
        })
    };
}

#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => {
        ::critical_section::with(|cs| {
            if let Some(uart) = $crate::UART_TX.borrow_ref_mut(cs).as_mut() {
                let _ = ::ufmt::uwrite!(uart, $($arg)*);
            }
        })
    };
}

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

#[embassy_executor::task]
async fn serial_read(
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut buf = [0u8; 64];
    loop {
        match rx.read(&mut buf).await {
            Ok(_) => {
                serial_println!("-- Serial: OK");
            }
            Err(_err) => {
                serial_println!("-- Serial: ERROR");
            }
        }
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let mut config = ch32_hal::Config::default();
    config.rcc = ch32_hal::rcc::Config::SYSCLK_FREQ_48MHZ_HSI;
    let p = ch32_hal::init(config);

    // UART
    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (mut uart_tx, uart_rx) = match usart::Uart::new(
        p.USART1,
        p.PD1, // RX
        p.PD0, // TX
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

    for _ in 0..10 {
        let _ = uart_tx.write(b"AAAAAAAAAABBBBBBBBBB\r\n").await;
    }

    // Create formatted writer and store in mutex
    let uart = UartFmt::new(uart_tx);
    critical_section::with(|cs| {
        UART_TX.borrow(cs).replace(Some(uart));
    });

    // Spawn serial reader task
    spawner.spawn(serial_read(uart_rx).unwrap());

    let reset = ch32_hal::pac::RCC.rstsckr().read().0;
    clear_reset();

    serial_println!("\n>> CH32V003 <<\n");
    serial_println!("RESET: 0x{:x}", reset);

    // LED
    let mut led = Output::new(p.PC3, Level::Low, Default::default());
    let mut i: u32 = 0;
    loop {
        serial_println!(">> count = {}", i);
        led.toggle();
        i += 1;
        Timer::after_millis(500).await;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
