use ch32_hal::i2c::I2c;
use ch32_hal::mode::Blocking;
use ch32_hal::peripherals::I2C1;

use portable_atomic::Ordering;

use crate::parse::{parse_hex, parse_u32};
use crate::serial::{
    serial_write, serial_write_fixed, serial_write_hex, serial_write_u32, ECHO, LINE_CHANNEL,
};
use crate::serial_fmt;
use crate::{
    IMON, LED_STATE, OFF_TIMER, ON_TIMER, POWER_OFF_DELAY, POWER_STATE, PS_ALARM, PS_OK, UPTIME,
    VREF,
};

const CMDS: &[&str] = &[
    "STATUS", "I2C", "ECHO", "POWER", "CANCEL", "DELAY", "DPS", "STACK",
];
const CMDS_I2C: &[&str] = &["SCAN", "READ", "WRITE", "READ-REG"];
const CMDS_DPS: &[&str] = &["READ", "STATUS"];
const CMDS_ON_OFF: &[&str] = &["ON", "OFF"];

const DPS_ADDR: u8 = 0x5f; // I2c address for controller (not SMBus)

/// Read DPS register
/// Thanks to DrTune for reverse-engineering protocol - https://github.com/raplin/DPS-1200FB
fn dps_read(i2c: &mut I2c<'static, I2C1, Blocking>, cmd: u8) -> Result<u16, CmdError> {
    let cs = (!(cmd.wrapping_add(DPS_ADDR << 1))).wrapping_add(1);
    i2c.blocking_write(DPS_ADDR, &[cmd, cs])
        .map_err(|_| CmdError::I2c)?;
    let mut buf = [0u8; 3];
    i2c.blocking_read(DPS_ADDR, &mut buf)
        .map_err(|_| CmdError::I2c)?;
    if buf.iter().fold(0u8, |a, &b| a.wrapping_add(b)) != 0 {
        return Err(CmdError::Crc);
    }
    let raw = u16::from_le_bytes([buf[0], buf[1]]);
    Ok(raw)
}

const DPS_CMDS: &[(&[u8], u8, u8, &[u8])] = &[
    (b"On: ", 0x30, 1, b"s"),
    (b"Vin: ", 0x08, 5, b"V"),
    (b"Iin: ", 0x0a, 7, b"A"),
    (b"Pin: ", 0x0c, 1, b"W"),
    (b"Vout: ", 0x0e, 8, b"V"),
    (b"Iout: ", 0x10, 7, b"A"),
    (b"Pout: ", 0x12, 1, b"W"),
    (b"Text: ", 0x1a, 6, b"C"),
    (b"Tint: ", 0x1c, 6, b"C"),
    (b"Fan: ", 0x1e, 0, b"rpm"),
];

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
    Crc,
}

#[inline(never)]
fn write_error(e: CmdError) {
    match e {
        CmdError::Invalid => serial_write(b"!! ERR-CMD\r\n"),
        CmdError::I2c => serial_write(b"!! ERR-I2C\r\n"),
        CmdError::Crc => serial_write(b"!! ERR-CRCC\r\n"),
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
                                    .blocking_write_read(
                                        addr as u8,
                                        &[reg as u8],
                                        &mut buf[..len as usize],
                                    )
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
                Some(6) => {
                    // DPS
                    match it.next().and_then(|s| lookup(s, CMDS_DPS)) {
                        Some(0) => {
                            // DPS READ <CMD> [SCALE]
                            if let Some(cmd) = it.next().map(parse_u32).filter(|&len| len <= 127) {
                                let scale = it.next().map(parse_u32).unwrap_or(0) as u8;
                                match dps_read(&mut i2c, cmd as u8) {
                                    Ok(v) => {
                                        serial_write(b">> ");
                                        if scale == 0 {
                                            serial_write_u32(v as u32);
                                        } else {
                                            serial_write_fixed(v, scale, 3);
                                        }
                                        serial_write(b"\r\n");
                                    }
                                    Err(e) => write_error(e),
                                }
                            } else {
                                write_error(CmdError::Invalid)
                            }
                        }
                        Some(1) => {
                            // DPS STATUS
                            for (measure, cmd, scale, unit) in DPS_CMDS {
                                match dps_read(&mut i2c, *cmd) {
                                    Ok(v) => {
                                        serial_write(b">> ");
                                        serial_write(measure);
                                        if *scale == 0 {
                                            serial_write_u32(v as u32);
                                        } else {
                                            serial_write_fixed(v, *scale, 3);
                                        }
                                        serial_write(unit);
                                        serial_write(b" [");
                                        serial_write_hex((v >> 8) as u8);
                                        serial_write_hex(v as u8);
                                        serial_write(b"]");
                                        serial_write(b"\r\n");
                                        embassy_futures::yield_now().await;
                                    }
                                    Err(e) => write_error(e),
                                }
                            }
                        }
                        _ => write_error(CmdError::Invalid),
                    }
                }
                Some(7) => {
                    // STACK
                    crate::stack::print_stack();
                }
                Some(_) | None => write_error(CmdError::Invalid),
            }
            if ECHO.load(Ordering::Relaxed) {
                serial_write(b"## ");
            }
        }
    }
}
