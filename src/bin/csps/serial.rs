use ch32_hal::usart;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::pipe::Pipe;

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
    ($($arg:tt)*) => {{ let _ = ::ufmt::uwriteln!(&mut $crate::serial::TxSink, $($arg)*); }};
}

type CmdFn = fn(&str);

#[embassy_executor::task]
pub async fn serial_read(
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
    handler: CmdFn,
) {
    serial_println!("-- [[ CH32V003 ]] --");
    let mut buf = [0u8; 16];
    let mut line_buf = heapless::String::<64>::new();
    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                for &b in &buf[..n] {
                    match b {
                        b'\n' | b'\r' => {
                            handler(&line_buf);
                            line_buf.clear();
                        }
                        0x20..=0x7e => {
                            if line_buf.push(b as char).is_err() {
                                handler(&line_buf);
                                line_buf.clear();
                                let _ = line_buf.push(b as char);
                            }
                        }
                        _ => {} // ignore control/non-ASCII
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
pub async fn serial_write(
    mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut b = [0u8; 64];
    loop {
        let n = TX_PIPE.read(&mut b).await;
        let _ = tx.write(&b[..n]).await;
    }
}
