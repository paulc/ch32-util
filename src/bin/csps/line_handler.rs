use ch32_hal::i2c::I2c;
use ch32_hal::mode::Blocking;
use ch32_hal::peripherals::I2C1;

use embassy_time::Timer;

use portable_atomic::Ordering;

use crate::parse::{parse_hex, parse_u32};
use crate::serial;
use crate::{serial_print, serial_println};
use crate::{LED_STATE, POWER_STATE};

#[embassy_executor::task]
pub async fn line_handler(mut i2c: I2c<'static, I2C1, Blocking>) {
    #[cfg(feature = "debug")]
    {
        ch32_util::stack_info::paint_stack();
        ch32_util::stack_info::print_stack_info(b"LINE_HANDLER");
    }
    loop {
        let line = serial::LINE_CHANNEL.receive().await;
        let line = line.trim_ascii();
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next() {
                Some("STATUS") => {
                    serial_println!(">> POWER_STATE: {}", POWER_STATE.load(Ordering::Relaxed));
                    serial_println!(">> LED_STATE: {}", LED_STATE.load(Ordering::Relaxed));
                }
                Some("I2C") => match it.next() {
                    Some("SCAN") => {
                        serial_println!(">> Scan I2C bus: START");
                        for addr in 1..=127 {
                            let mut buf = [0u8; 1];
                            if i2c.blocking_read(addr, &mut buf).is_ok() {
                                serial_println!(">>>> Found I2C device at address: 0x{:02x}", addr);
                                // ALlow serial_write buffer to clear
                                Timer::after_millis(10).await;
                            }
                        }
                        serial_println!(">> Scan I2C bus: DONE");
                    }
                    // READ <ADDRESS> <LEN>
                    Some("READ") => {
                        let mut buf = [0_u8; 16];
                        let addr = it.next().map(parse_u32).filter(|&len| len <= 127);
                        let len = it.next().map(parse_u32).filter(|&len| len <= 16);
                        if let (Some(addr), Some(len)) = (addr, len) {
                            serial_println!(">> READ 0x{:x} {}", addr, len);
                            if i2c
                                .blocking_read(addr as u8, &mut buf[..len as usize])
                                .is_ok()
                            {
                                serial_print!("== ");
                                for i in 0..len {
                                    serial_print!("{:02x}", buf[i as usize]);
                                }
                                serial_println!("");
                            } else {
                                serial_println!("!! READ ERROR");
                            }
                        } else {
                            serial_println!("!! INVALID COMMAND");
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
                            serial_println!(">> WRITE 0x{:x} {:?} {}", addr, buf, len);
                            if i2c.blocking_write(addr as u8, &buf[..len]).is_ok() {
                                serial_println!("== WRITE OK");
                            } else {
                                serial_println!("!! READ ERROR");
                            }
                        } else {
                            serial_println!("!! INVALID COMMAND");
                        }
                    }
                    _ => {}
                },
                Some("ECHO") => {
                    match it.next() {
                        Some("ON") => serial::ECHO.store(true, Ordering::Relaxed),
                        Some("OFF") => serial::ECHO.store(false, Ordering::Relaxed),
                        _ => {}
                    }
                    serial_println!(
                        ">> ECHO {}",
                        if serial::ECHO.load(Ordering::Relaxed) {
                            "ON"
                        } else {
                            "OFF"
                        }
                    );
                }
                Some(s) => serial_println!("!! ERROR: <{}> [{}]", s, s.len()),
                None => {}
            }
            if serial::ECHO.load(Ordering::Relaxed) {
                serial_print!("## ");
            }
        }
    }
}
