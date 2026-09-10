use ch32_hal::i2c::I2c;
use ch32_hal::mode::Blocking;
use ch32_hal::peripherals::I2C1;

use portable_atomic::Ordering;

use crate::parse::{parse_hex, parse_u32};
use crate::serial::{serial_write, serial_write_hex, ECHO, LINE_CHANNEL};
use crate::serial_fmt;
use crate::{LED_STATE, POWER_STATE};

#[embassy_executor::task]
pub async fn line_handler(mut i2c: I2c<'static, I2C1, Blocking>) {
    #[cfg(feature = "debug")]
    {
        ch32_util::stack_info::paint_stack();
        ch32_util::stack_info::print_stack_info(b"LINE_HANDLER");
    }
    loop {
        let line = LINE_CHANNEL.receive().await;
        let line = line.trim_ascii();
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next() {
                Some("STATUS") => {
                    serial_fmt!(
                        b">> POWER_STATE: ",
                        BOOL(POWER_STATE.load(Ordering::Relaxed)),
                        b"\r\n>> LED STATE: ",
                        U32(LED_STATE.load(Ordering::Relaxed) as u32),
                        b"\r\n"
                    );
                }
                Some("I2C") => match it.next() {
                    Some("SCAN") => {
                        for addr in 1..=127 {
                            let mut buf = [0u8; 1];
                            if i2c.blocking_read(addr, &mut buf).is_ok() {
                                serial_fmt!(b">> I2C DEVICE: 0x", HEX(addr as u8), b"\r\n");
                            }
                        }
                    }
                    // READ <ADDRESS> <LEN>
                    Some("READ") => {
                        let mut buf = [0_u8; 16];
                        let addr = it.next().map(parse_u32).filter(|&len| len <= 127);
                        let len = it.next().map(parse_u32).filter(|&len| len <= 16);
                        if let (Some(addr), Some(len)) = (addr, len) {
                            serial_fmt!(
                                b">> I2C READ 0x",
                                HEX(addr as u8),
                                b" ",
                                U32(len),
                                b"\r\n"
                            );
                            if i2c
                                .blocking_read(addr as u8, &mut buf[..len as usize])
                                .is_ok()
                            {
                                serial_write(b"== ");
                                for i in 0..len {
                                    serial_write_hex(buf[i as usize]);
                                }
                                serial_write(b"\r\n");
                            } else {
                                serial_write(b"!! ERR-I2C\r\n");
                            }
                        } else {
                            serial_write(b"!! ERR-CMD\r\n");
                        }
                    }
                    // WRITE <ADDRESS> <DATA>
                    Some("WRITE") => {
                        let addr = it.next().map(parse_u32).filter(|&len| len <= 127);
                        let data = it
                            .next()
                            .filter(|&data| data.len() <= 32)
                            .and_then(parse_hex);
                        if let (Some(addr), Some((buf, len))) = (addr, data) {
                            if i2c.blocking_write(addr as u8, &buf[..len]).is_ok() {
                                serial_write(b"== WRITE OK\r\n");
                            } else {
                                serial_write(b"!! ERR-I2C\r\n");
                            }
                        } else {
                            serial_write(b"!! ERR-CMD\r\n");
                        }
                    }
                    _ => {}
                },
                Some("ECHO") => {
                    match it.next() {
                        Some("ON") => ECHO.store(true, Ordering::Relaxed),
                        Some("OFF") => ECHO.store(false, Ordering::Relaxed),
                        _ => {}
                    }
                    serial_write(b">> ECHO ");
                    serial_write(if ECHO.load(Ordering::Relaxed) {
                        b"ON\r\n"
                    } else {
                        b"OFF\r\n"
                    });
                }
                Some(_) => serial_write(b"!! ERR-CMD\r\n"),
                None => {}
            }
            if ECHO.load(Ordering::Relaxed) {
                serial_write(b"## ");
            }
        }
    }
}
