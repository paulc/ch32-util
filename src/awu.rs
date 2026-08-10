//! Minimal PWR / auto-wakeup (AWU) wrapper for the CH32V003.
//!
//! ch32-metapac defines `pwr_v00x` but does not instantiate PWR for the V003
//! chip, so this drives the registers directly. Addresses are from section 2.4
//! of the CH32V003 reference manual:
//!
//! ```text
//! R32_PWR_CTLR    0x40007000  Power control            reset 0x00000000
//! R32_PWR_CSR     0x40007004  Power control/status     reset 0x00000000
//! R32_PWR_AWUCSR  0x40007008  Auto-wakeup control      reset 0x00000000
//! R32_PWR_AWUWR   0x4000700C  Auto-wakeup window       reset 0x0000003F
//! R32_PWR_AWUPSC  0x40007010  Auto-wakeup prescaler    reset 0x00000000
//! ```
//!
//! The AWU counter is clocked by the LSI (nominally 128 kHz), an untrimmed RC
//! oscillator with wide tolerance over temperature and supply. Treat all
//! periods as approximate.
//!
//! IMPORTANT: the V003 does not stop the IWDG in standby. If you sleep for
//! longer than the watchdog period you will be reset by the watchdog rather
//! than woken by the AWU. The two features are effectively mutually exclusive
//! on this part unless the sleep is short.

use core::ptr::{read_volatile, write_volatile};

const PWR_BASE: usize = 0x4000_7000;
const CTLR: *mut u32 = PWR_BASE as *mut u32;
const AWUCSR: *mut u32 = (PWR_BASE + 0x08) as *mut u32;
const AWUWR: *mut u32 = (PWR_BASE + 0x0C) as *mut u32;
const AWUPSC: *mut u32 = (PWR_BASE + 0x10) as *mut u32;

const CTLR_PDDS: u32 = 1 << 1; // 1 = standby, 0 = sleep
const AWUCSR_AWUEN: u32 = 1 << 1;

/// QingKe PFIC system control register. Bit 2 (SLEEPDEEP) must be set for
/// `wfi` to reach standby rather than sleep. Verify this address against the
/// PFIC chapter of the reference manual before relying on it.
const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
const SCTLR_SLEEPDEEP: u32 = 1 << 2;

const LSI_HZ: u32 = 128_000;

/// AWU counter prescaler. The encoding is irregular — it is not a simple
/// power-of-two exponent, and the top two values jump to /10240 and /61440.
#[derive(Clone, Copy)]
#[repr(u32)]
pub enum Prescaler {
    /// /1 — up to 0.5 ms
    Div1 = 0b0001,
    /// /2 — up to 1 ms
    Div2 = 0b0010,
    /// /4 — up to 2 ms
    Div4 = 0b0011,
    /// /8 — up to 4 ms
    Div8 = 0b0100,
    /// /16 — up to 8 ms
    Div16 = 0b0101,
    /// /32 — up to 16 ms
    Div32 = 0b0110,
    /// /64 — up to 32 ms
    Div64 = 0b0111,
    /// /128 — up to 64 ms
    Div128 = 0b1000,
    /// /256 — up to 128 ms
    Div256 = 0b1001,
    /// /512 — up to 256 ms
    Div512 = 0b1010,
    /// /1024 — up to 512 ms
    Div1024 = 0b1011,
    /// /2048 — up to 1.02 s
    Div2048 = 0b1100,
    /// /4096 — up to 2.05 s
    Div4096 = 0b1101,
    /// /10240 — up to 5.12 s
    Div10240 = 0b1110,
    /// /61440 — up to 30.7 s
    Div61440 = 0b1111,
}

impl Prescaler {
    const fn divider(self) -> u32 {
        match self {
            Prescaler::Div1 => 1,
            Prescaler::Div2 => 2,
            Prescaler::Div4 => 4,
            Prescaler::Div8 => 8,
            Prescaler::Div16 => 16,
            Prescaler::Div32 => 32,
            Prescaler::Div64 => 64,
            Prescaler::Div128 => 128,
            Prescaler::Div256 => 256,
            Prescaler::Div512 => 512,
            Prescaler::Div1024 => 1024,
            Prescaler::Div2048 => 2048,
            Prescaler::Div4096 => 4096,
            Prescaler::Div10240 => 10240,
            Prescaler::Div61440 => 61440,
        }
    }
}

/// Why a requested wake-up interval could not be configured.
///
/// Deliberately has no `Debug` impl: deriving one would pull `core::fmt` into
/// the binary via `Result::unwrap`. Match on this rather than unwrapping.
#[derive(Clone, Copy)]
pub enum Error {
    /// Interval is shorter than one count at this prescaler.
    TooShort,
    /// Interval needs more than 64 counts at this prescaler.
    TooLong,
}

/// Configure the auto-wakeup timer. The window value is compared against an
/// up-counter; a wake-up fires when they match.
///
/// With literal arguments the arithmetic constant-folds away; a runtime
/// `interval_ms` pulls in a software divide (`__udivsi3`) on this core.
pub fn awu_configure(psc: Prescaler, interval_ms: u32) -> Result<(), Error> {
    // interval = (AWUWR + 1) * divider / LSI_HZ
    let ticks_per_ms = LSI_HZ / 1000; // 128
    if interval_ms > u32::MAX / ticks_per_ms {
        return Err(Error::TooLong);
    }
    let counts = (interval_ms * ticks_per_ms) / psc.divider();
    if counts == 0 {
        return Err(Error::TooShort);
    }
    if counts > 64 {
        return Err(Error::TooLong);
    }

    unsafe {
        write_volatile(AWUPSC, psc as u32);
        write_volatile(AWUWR, (counts - 1) & 0x3F);
    }
    Ok(())
}

/// Nominal interval for a prescaler and window value, in milliseconds.
pub const fn actual_ms(psc: Prescaler, window: u8) -> u32 {
    ((window as u32 + 1) * psc.divider()) / (LSI_HZ / 1000)
}

#[inline]
pub fn awu_enable() {
    unsafe { write_volatile(AWUCSR, AWUCSR_AWUEN) };
}

#[inline]
pub fn awu_disable() {
    unsafe { write_volatile(AWUCSR, 0) };
}

/// Enter sleep mode: the core clock stops, peripherals keep running, and any
/// enabled interrupt wakes it. Execution resumes at the instruction after the
/// `wfi`.
#[inline]
pub fn sleep() {
    unsafe {
        write_volatile(CTLR, read_volatile(CTLR) & !CTLR_PDDS);
        write_volatile(PFIC_SCTLR, read_volatile(PFIC_SCTLR) & !SCTLR_SLEEPDEEP);
        core::arch::asm!("wfi");
    }
}

/// Enter standby mode. Much lower current than `sleep`, but most of the chip
/// is powered down.
///
/// Call `awu_configure` and `awu_enable` first, or nothing will wake it.
///
/// NOTE: whether the V003 resumes after the `wfi` or restarts through the
/// reset vector on wake is worth confirming on hardware — print a marker at
/// boot and see whether it reappears. If it restarts, RAM contents are not
/// guaranteed and any state must be recomputed.
#[inline]
pub fn standby() {
    unsafe {
        write_volatile(CTLR, read_volatile(CTLR) | CTLR_PDDS);
        write_volatile(PFIC_SCTLR, read_volatile(PFIC_SCTLR) | SCTLR_SLEEPDEEP);
        core::arch::asm!("wfi");
    }
}
