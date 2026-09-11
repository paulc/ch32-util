use ch32_hal::i2c::I2c;
use ch32_hal::mode::Blocking;
use ch32_hal::peripherals::I2C1;

use portable_atomic::Ordering;

use crate::parse::{parse_hex, parse_u32};
use crate::serial::{serial_write, serial_write_hex, serial_write_u32, ECHO, LINE_CHANNEL};
use crate::serial_fmt;
use crate::{
    IMON, LED_STATE, OFF_TIMER, ON_TIMER, POWER_OFF_DELAY, POWER_STATE, PS_ALARM, PS_OK, UPTIME,
    VREF,
};

const CMDS: &[&str] = &["STATUS", "I2C", "ECHO", "POWER", "CANCEL", "DELAY"];
const CMDS_I2C: &[&str] = &["SCAN", "READ", "WRITE", "READ-REG"];
const CMDS_ON_OFF: &[&str] = &["ON", "OFF"];

#[inline(never)]
fn lookup(s: &str, table: &[&str]) -> Option<usize> {
    table.iter().position(|&t| t == s)
}

#[inline(never)]
fn status_line(label: &[u8], v: u32) {
    serial_write(b">> ");
    serial_write(label);
    serial_write_u32(v);
    serial_write(b"\r\n");
}

enum CmdError {
    Invalid,
    I2c,
}

#[inline(never)]
fn write_error(e: CmdError) {
    match e {
        CmdError::Invalid => serial_write(b"!! ERR-CMD\r\n"),
        CmdError::I2c => serial_write(b"!! ERR-I2C\r\n"),
    }
}

#[embassy_executor::task]
pub async fn line_handler(mut i2c: I2c<'static, I2C1, Blocking>) {
    loop {
        let line = LINE_CHANNEL.receive().await;
        let line = line.trim_ascii();
        if !line.is_empty() {
            let mut it = line.split_ascii_whitespace();
            match it.next().and_then(|s| lookup(s, CMDS)) {
                Some(0) => {
                    // STATUS
                    status_line(b"UPTIME: ", UPTIME.load(Ordering::Relaxed));
                    status_line(b"ON_TIMER: ", ON_TIMER.load(Ordering::Relaxed));
                    status_line(b"OFF_TIMER: ", OFF_TIMER.load(Ordering::Relaxed));
                    status_line(
                        b"POWER_OFF_DELAY: ",
                        POWER_OFF_DELAY.load(Ordering::Relaxed),
                    );
                    status_line(b"POWER_STATE: ", POWER_STATE.load(Ordering::Relaxed) as u32);
                    status_line(b"LED_STATE: ", LED_STATE.load(Ordering::Relaxed) as u32);
                    status_line(b"PS_OK: ", PS_OK.load(Ordering::Relaxed) as u32);
                    status_line(b"PS_ALARM: ", PS_ALARM.load(Ordering::Relaxed) as u32);
                    status_line(b"VREF: ", VREF.load(Ordering::Relaxed) as u32);
                    status_line(b"IMON: ", IMON.load(Ordering::Relaxed) as u32);
                }
                Some(1) => {
                    // I2C
                    match it.next().and_then(|s| lookup(s, CMDS_I2C)) {
                        Some(0) => {
                            // SCAN
                            for addr in 1..=127 {
                                let mut buf = [0u8; 1];
                                if i2c.blocking_read(addr, &mut buf).is_ok() {
                                    serial_fmt!(b">> I2C DEVICE: 0x", HEX(addr as u8), b"\r\n");
                                }
                            }
                        }
                        Some(1) => {
                            // READ <ADDRESS> <LEN>
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
                                    serial_write(b">> ");
                                    for i in 0..len {
                                        serial_write_hex(buf[i as usize]);
                                    }
                                    serial_write(b"\r\n");
                                } else {
                                    write_error(CmdError::Invalid);
                                }
                            } else {
                                write_error(CmdError::Invalid);
                            }
                        }
                        Some(2) => {
                            // WRITE <ADDRESS> <DATA>
                            let addr = it.next().map(parse_u32).filter(|&len| len <= 127);
                            let data = it
                                .next()
                                .filter(|&data| data.len() <= 32)
                                .and_then(parse_hex);
                            if let (Some(addr), Some((buf, len))) = (addr, data) {
                                if i2c.blocking_write(addr as u8, &buf[..len]).is_ok() {
                                    serial_write(b">> WRITE OK\r\n");
                                } else {
                                    write_error(CmdError::I2c);
                                }
                            } else {
                                write_error(CmdError::Invalid);
                            }
                        }
                        Some(3) => {
                            // READ-REG <ADDRESS> <REGISTER> <LENGTH>
                            let mut buf = [0_u8; 16];
                            let addr = it.next().map(parse_u32).filter(|&len| len <= 127);
                            let reg = it.next().map(parse_u32).filter(|&len| len <= 255);
                            let len = it.next().map(parse_u32).filter(|&len| len <= 16);
                            if let (Some(addr), Some(reg), Some(len)) = (addr, reg, len) {
                                if i2c
                                    .blocking_write(addr as u8, &[reg as u8])
                                    .and_then(|_| {
                                        i2c.blocking_read(addr as u8, &mut buf[..len as usize])
                                    })
                                    .is_ok()
                                {
                                    serial_write(b">> ");
                                    for i in 0..len {
                                        serial_write_hex(buf[i as usize]);
                                    }
                                    serial_write(b"\r\n");
                                } else {
                                    write_error(CmdError::I2c);
                                }
                            } else {
                                write_error(CmdError::Invalid);
                            }
                        }
                        _ => {}
                    }
                }
                Some(2) => {
                    // ECHO
                    match it.next().and_then(|s| lookup(s, CMDS_ON_OFF)) {
                        Some(0) => {
                            // ECHO ON
                            ECHO.store(true, Ordering::Relaxed)
                        }
                        Some(1) => {
                            // ECHO OFF
                            ECHO.store(false, Ordering::Relaxed)
                        }
                        _ => {}
                    }
                    serial_write(b">> ECHO ");
                    serial_write(if ECHO.load(Ordering::Relaxed) {
                        b"ON\r\n"
                    } else {
                        b"OFF\r\n"
                    });
                }
                Some(3) => {
                    // POWER
                    match it.next().and_then(|s| lookup(s, CMDS_ON_OFF)) {
                        Some(0) => {
                            // POWER ON [SECS]
                            let secs = it.next().map(parse_u32).unwrap_or(0).min(86_400);
                            let on_time = UPTIME.load(Ordering::Relaxed) + secs;
                            ON_TIMER.store(on_time, Ordering::Relaxed);
                            serial_fmt!(b">> POWER ON: ", U32(on_time), b"\r\n");
                        }
                        Some(1) => {
                            // POWER OFF [SECS]
                            // Minimum of 5 sec power-off timer
                            let secs = it
                                .next()
                                .map(parse_u32)
                                .unwrap_or(0)
                                .max(POWER_OFF_DELAY.load(Ordering::Relaxed))
                                .min(86_400);
                            let off_time = UPTIME.load(Ordering::Relaxed) + secs;
                            OFF_TIMER.store(off_time, Ordering::Relaxed);
                            serial_fmt!(b">> POWER OFF: ", U32(off_time), b"\r\n");
                        }
                        _ => write_error(CmdError::Invalid),
                    }
                }
                Some(4) => {
                    // CANCEL
                    match it.next().and_then(|s| lookup(s, CMDS_ON_OFF)) {
                        Some(0) => {
                            // CANCEL ON
                            ON_TIMER.store(0, Ordering::Relaxed);
                            serial_write(b">> OK\r\n");
                        }
                        Some(1) => {
                            // CANCEL OFF
                            OFF_TIMER.store(0, Ordering::Relaxed);
                            serial_write(b">> OK\r\n");
                        }
                        _ => write_error(CmdError::Invalid),
                    }
                }
                Some(5) => {
                    // DELAY SECS
                    if let Some(secs) = it.next().map(parse_u32) {
                        // Max 5-min power off delay
                        POWER_OFF_DELAY.store(secs.min(600), Ordering::Relaxed);
                    } else {
                        write_error(CmdError::Invalid)
                    }
                }
                Some(_) | None => write_error(CmdError::Invalid),
            }
            if ECHO.load(Ordering::Relaxed) {
                serial_write(b"## ");
            }
        }
    }
}
