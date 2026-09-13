use ch32_hal::usart;

use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::pipe::Pipe;

use portable_atomic::{AtomicBool, Ordering};

const HEX: &[u8; 16] = b"0123456789abcdef";

pub static TX_PIPE: Pipe<CriticalSectionRawMutex, 256> = Pipe::new();
pub static LINE_CHANNEL: Channel<CriticalSectionRawMutex, heapless::String<64>, 1> = Channel::new();
pub static ECHO: AtomicBool = AtomicBool::new(false);
pub static UCASE: AtomicBool = AtomicBool::new(false);

fn push(mut data: &[u8]) -> Result<(), ()> {
    while !data.is_empty() {
        match TX_PIPE.try_write(data) {
            Ok(n) => data = &data[n..],
            Err(_) => return Err(()), // pipe full
        }
    }
    Ok(())
}

#[inline(never)]
pub fn serial_write(data: &[u8]) {
    // Drops data if TX_PIPE full
    let _ = push(data);
}

#[inline(never)]
#[allow(unused)]
/// Write with back-pressure in case we are overflowing TX_PIPE
/// (adds async yield point so use sparingly)
pub async fn serial_write_async(mut data: &[u8]) {
    while !data.is_empty() {
        let n = TX_PIPE.write(data).await;
        data = &data[n..];
    }
}

#[inline(never)]
pub fn serial_write_hex(v: u8) {
    // Drops data if TX_PIPE full
    let _ = push(&[HEX[(v >> 4) as usize], HEX[(v & 0x0f) as usize]]);
}

#[inline(never)]
pub fn serial_write_u32(v: u32) {
    let mut buf = [b'0'; 10];
    let mut n = v;
    let mut idx = 10;

    // Fill digits from right to left
    while n > 0 && idx > 0 {
        idx -= 1;
        buf[idx] = b'0' + (n % 10) as u8;
        n /= 10;
    }
    let zeros = buf
        .iter()
        .position(|&b| b != b'0')
        .unwrap_or(buf.len())
        .min(9);

    // Drops data if TX_PIPE full
    let _ = push(&buf[zeros..]);
}

/// Print a fixed-point value: `raw` with `scale` fractional bits.
/// serial_write_fixed(0x0C4C, 8, 3) -> "12.296"
#[inline(never)]
pub fn serial_write_fixed(raw: u16, scale: u8, digits: u8) {
    serial_write_u32((raw >> scale) as u32);
    if digits == 0 || scale == 0 {
        return;
    }
    let mask = (1u32 << scale) - 1;
    let mut f = raw as u32 & mask;
    let mut out = [b'.'; 5];
    let n = (digits as usize).min(4);
    for i in 0..n {
        f *= 10;
        out[i + 1] = b'0' + (f >> scale) as u8;
        f &= mask;
    }
    serial_write(&out[..n + 1]);
}

// Helper macro for serial_write...
#[macro_export]
macro_rules! serial_fmt {
    // Base case
    () => {};

    // Non-last items: comma must be present.
    (U32($e:expr), $($rest:tt)*) => {
        $crate::serial::serial_write_u32($e);
        $crate::serial_fmt!($($rest)*);
    };
    (HEX($e:expr), $($rest:tt)*) => {
        $crate::serial::serial_write_hex($e);
        $crate::serial_fmt!($($rest)*);
    };
    ($e:expr, $($rest:tt)*) => {
        $crate::serial::serial_write($e);
        $crate::serial_fmt!($($rest)*);
    };

    // Last item: no trailing comma.
    (U32($e:expr)) => { $crate::serial::serial_write_u32($e); };
    (HEX($e:expr)) => { $crate::serial::serial_write_hex($e); };
    ($e:expr) => { $crate::serial::serial_write($e); };
}

/*
///
/// Implement uFmt for formatted serial output - removed to save space
///
pub struct TxSink;

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

#[macro_export]
macro_rules! serial_print {
    ($($arg:tt)*) => {{ let _ = ::ufmt::uwrite!(&mut $crate::serial::TxSink, $($arg)*); }};
}
*/

#[embassy_executor::task]
pub async fn serial_read_task(
    mut rx: usart::UartRx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut buf = [0u8; 64];
    let mut line_buf = heapless::String::<64>::new();
    let mut crlf = false;
    loop {
        match rx.read_until_idle(&mut buf).await {
            Ok(n) => {
                for &b in &buf[..n] {
                    match b {
                        b'\n' | b'\r' => {
                            if !crlf {
                                if ECHO.load(Ordering::Relaxed) {
                                    serial_write(b"\r\n");
                                }
                                // Send to LINE_CHANNEL - drop if channel full
                                let _ = LINE_CHANNEL.try_send(line_buf.clone());
                                line_buf.clear();
                                crlf = true;
                            }
                        }
                        0x1b => {
                            // ESC
                            line_buf.clear();
                            if ECHO.load(Ordering::Relaxed) {
                                serial_write(b"\r\n## ");
                            }
                        }
                        0x20..=0x7e => {
                            // ASCII
                            // Convert to uppercase
                            let b = if UCASE.load(Ordering::Relaxed) && b >= b'a' && b <= b'z' {
                                b - 0x20
                            } else {
                                b
                            };
                            if ECHO.load(Ordering::Relaxed) {
                                serial_write(&[b]);
                            }
                            if line_buf.push(b as char).is_err() {
                                // Drop line if exceeds line_buf
                                line_buf.clear();
                                if ECHO.load(Ordering::Relaxed) {
                                    serial_write(b"\r\n!! ERR-LINE\r\n## ");
                                }
                            }
                            crlf = false;
                        }
                        _ => {} // ignore control/non-ASCII
                    }
                }
            }
            Err(_) => serial_write(b"\r\n!! ERR-SERIAL\r\n"),
        }
    }
}

#[embassy_executor::task]
pub async fn serial_write_task(
    mut tx: usart::UartTx<'static, ch32_hal::peripherals::USART1, ch32_hal::mode::Async>,
) {
    let mut b = [0u8; 64];
    loop {
        let n = TX_PIPE.read(&mut b).await;
        let _ = tx.write(&b[..n]).await;
    }
}
