use core::arch::asm;
use core::ptr::{read_volatile, write_volatile};

use crate::sdi_write::FormatHex;

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

pub fn print_stack_info(key: &[u8]) {
    let (sp, hwm) = stack_headroom();
    crate::sdi_writeln!(
        b"STACK:",
        key,
        &(sp as u32).fmt_hex(),
        &(hwm as u32).fmt_hex()
    );
}
