#![no_std]
#![no_main]

use ch32_hal::bind_interrupts;
use ch32_hal::gpio::{Level, Output};
use ch32_hal::usart;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;
use embassy_time::Timer;

bind_interrupts!(struct Irqs {
    USART1 => ch32_hal::usart::InterruptHandler<ch32_hal::peripherals::USART1>;
});

pub static TX_PIPE: Pipe<CriticalSectionRawMutex, 64> = Pipe::new();

pub struct TxSink;

fn push(mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        match TX_PIPE.try_write(data) {
            Ok(n) => data = &data[n..],
            Err(_) => return Err(()), // pipe full
        }
    }
    Ok(())
}

impl ufmt::uWrite for TxSink {
    type Error = ();
    fn write_str(&mut self, s: &str) -> Result<(), ()> {
        let mut rest = s.as_bytes();
        // Translate '\n' to '\r\n'
        while let Some(i) = rest.iter().position(|&b| b == b'\n') {
            let (head, tail) = rest.split_at(i);
            push(head)?;
            if head.last() != Some(&b'\r') {
                push(b"\r")?;
            }
            push(b"\n")?;
            rest = &tail[1..];
        }
        push(rest)
    }
}

#[macro_export]
macro_rules! serial_println {
    ($($arg:tt)*) => {{ let _ = ::ufmt::uwriteln!(&mut $crate::TxSink, $($arg)*); }};
}

async fn handle_cmd(line: &str) {
    let line = line.trim_ascii();
    if !line.is_empty() {
        let mut it = line.split_ascii_whitespace();
        match it.next() {
            Some("hello") => serial_println!("-- Hello >>{}<<", it.next().unwrap_or("There")),
            Some(cmd) => serial_println!("-- CMD: {}", cmd),
            None => {}
        }
    }
}

#[embassy_executor::task]
async fn serial_read(
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut buf = [0u8; 16];
    let mut line_buf = heapless::String::<64>::new();
    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                if n > 0 && buf[..n].is_ascii() {
                    let s = unsafe { str::from_utf8_unchecked(&buf[..n]) };
                    // Check if buffer full
                    if line_buf.push_str(s).is_err() {
                        handle_cmd(&line_buf).await;
                        line_buf.clear();
                    }
                    // Check for line
                    while let Some(i) = line_buf.find(['\n', '\r']) {
                        handle_cmd(&line_buf[..i].trim_ascii()).await;
                        line_buf.drain(..=i);
                    }
                }
            }
            Err(e) => match e {
                usart::Error::Overrun => {
                    serial_println!("-- ERR overrun");
                }
                usart::Error::Framing => {
                    serial_println!("-- ERR framing");
                }
                usart::Error::Noise => {
                    serial_println!("-- ERR noise");
                }
                _ => {
                    serial_println!("-- ERR other");
                }
            },
        }
    }
}

#[embassy_executor::task]
async fn serial_write(
    mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut b = [0u8; 64];
    loop {
        let n = TX_PIPE.read(&mut b).await;
        let _ = tx.write(&b[..n]).await;
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(spawner: Spawner) -> ! {
    let config = ch32_hal::Config::default();
    let p = ch32_hal::init(config);

    let uart_tx = p.PC0;
    let uart_rx = p.PC1;

    let uart_config = usart::Config::default(); // 115200,N,8,1
    let (uart_tx, uart_rx) = match usart::Uart::new(
        p.USART1,
        uart_rx,
        uart_tx,
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

    // Spawn serial reader/writer tasks
    spawner.spawn(serial_read(uart_rx).unwrap());
    spawner.spawn(serial_write(uart_tx).unwrap());

    // LED
    let mut led = Output::new(p.PC3, Level::Low, Default::default());
    let mut count = 0_u32;
    loop {
        led.toggle();
        serial_println!(">> Count: {}", count);
        Timer::after_millis(500).await;
        count += 1;
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}
