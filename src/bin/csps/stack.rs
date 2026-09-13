use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

use crate::serial::{serial_write, serial_write_hex, serial_write_u32};

const PAINT: u32 = 0xAAAA_AAAA;
const RAM_TOP: usize = 0x2000_0800; // 2 KB SRAM base 0x2000_0000

unsafe extern "C" {
    static mut _ebss: u32;
}

#[inline(never)]
pub fn paint_stack() {
    let sp: usize;
    unsafe { asm!("mv {}, sp", out(reg) sp) };

    let bottom = (&raw const _ebss as usize + 3) & !3;
    let top = (sp - 64) & !3; // headroom for this frame

    let mut p = bottom;
    while p < top {
        unsafe { write_volatile(p as *mut u32, PAINT) };
        p += 4;
    }
}

// SP, HWM
pub fn stack_headroom() -> (usize, usize) {
    let bottom = (&raw const _ebss as usize + 3) & !3;
    let mut p = bottom;
    while p < RAM_TOP {
        if unsafe { read_volatile(p as *const u32) } != PAINT {
            break;
        }
        p += 4;
    }
    let hwm = p - bottom; // bytes of margin never touched
    let sp: usize;
    unsafe { core::arch::asm!("mv {}, sp", out(reg) sp) };
    (sp, hwm)
}

#[inline(never)]
pub fn print_stack() {
    let (sp, hwm) = crate::stack::stack_headroom();
    serial_write(b">> SP/HWM: 0x");
    for b in (sp as u32).to_be_bytes() {
        serial_write_hex(b);
    }
    serial_write(b" ");
    serial_write_u32(hwm as u32);
    serial_write(b"\r\n");
}
