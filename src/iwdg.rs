//! Minimal IWDG (independent watchdog) wrapper for the CH32V003.
//!
//! ch32-metapac has no IWDG definition for the V003 (it does for the V006),
//! so this drives the registers directly. Addresses and reset values are from
//! section 4.3 of the CH32V003 reference manual:
//!
//! ```text
//! R16_IWDG_CTLR   0x40003000  Control     reset 0x0000
//! R16_IWDG_PSCR   0x40003004  Prescaler   reset 0x0000
//! R16_IWDG_RLDR   0x40003008  Reload      reset 0x0FFF
//! R16_IWDG_STATR  0x4000300C  Status      reset 0x0000
//! ```
//!
//! The IWDG runs from the LSI RC oscillator, so it keeps counting even if the
//! main clock tree fails. Once started it cannot be stopped — including at a
//! debugger breakpoint, which will reset the part mid-session unless the
//! DBGMCU freeze bit is set.
//!
//! LSI is nominally 128 kHz but is an untrimmed RC oscillator with wide
//! tolerance over temperature and supply. All periods below are nominal;
//! feed at well under half the configured period.

use core::ptr::{read_volatile, write_volatile};

const IWDG_BASE: usize = 0x4000_3000;
const CTLR: *mut u16 = IWDG_BASE as *mut u16;
const PSCR: *mut u16 = (IWDG_BASE + 0x04) as *mut u16;
const RLDR: *mut u16 = (IWDG_BASE + 0x08) as *mut u16;
const STATR: *const u16 = (IWDG_BASE + 0x0C) as *const u16;

const KEY_UNLOCK: u16 = 0x5555;
const KEY_FEED: u16 = 0xAAAA;
const KEY_START: u16 = 0xCCCC;

const PVU: u16 = 1 << 0; // prescaler update in progress
const RVU: u16 = 1 << 1; // reload update in progress

const LSI_HZ: u32 = 128_000;
const TICKS_PER_MS: u32 = LSI_HZ / 1000; // 128

/// Bound on the STATR busy-wait. PVU/RVU clear within a few LSI periods, so
/// this only trips if the LSI never starts.
const STATR_SPIN_LIMIT: u32 = 100_000;

/// LSI prescaler. The PR field encodes an exponent: divider = 4 << PR.
///
/// The counter is 12 bits, so each prescaler reaches a period of
/// `divider / 128` ms (one count) up to `4096 * divider / 128` ms.
#[derive(Clone, Copy)]
#[repr(u16)]
pub enum Prescaler {
    /// /4 — 0.03 ms to 128 ms
    Div4 = 0,
    /// /8 — 0.06 ms to 256 ms
    Div8 = 1,
    /// /16 — 0.125 ms to 512 ms
    Div16 = 2,
    /// /32 — 0.25 ms to 1024 ms
    Div32 = 3,
    /// /64 — 0.5 ms to 2048 ms
    Div64 = 4,
    /// /128 — 1 ms to 4096 ms
    Div128 = 5,
    /// /256 — 2 ms to 8192 ms
    Div256 = 6,
}

impl Prescaler {
    const fn divider(self) -> u32 {
        4 << (self as u16)
    }
}

/// Why the watchdog could not be started.
///
/// Deliberately has no `Debug` impl: deriving one would pull `core::fmt` into
/// the binary via `Result::unwrap`. Match on this rather than unwrapping.
#[derive(Clone, Copy)]
pub enum Error {
    /// Period is shorter than one count at this prescaler.
    TooShort,
    /// Period needs more than 4096 counts at this prescaler.
    TooLong,
    /// PSCR/RLDR stayed busy — the LSI is probably not running.
    LsiTimeout,
}

fn wait_clear(flag: u16) -> Result<(), Error> {
    let mut spins = 0u32;
    while unsafe { read_volatile(STATR) } & flag != 0 {
        spins += 1;
        if spins > STATR_SPIN_LIMIT {
            return Err(Error::LsiTimeout);
        }
    }
    Ok(())
}

pub struct Watchdog;

impl Watchdog {
    /// Start the watchdog with the given prescaler and nominal period.
    ///
    /// Cannot be stopped once started. With literal arguments the arithmetic
    /// and range checks constant-fold away; a runtime `period_ms` pulls in a
    /// software divide (`__udivsi3`) on this core.
    pub fn start(psc: Prescaler, period_ms: u32) -> Result<Self, Error> {
        if period_ms > u32::MAX / TICKS_PER_MS {
            return Err(Error::TooLong);
        }
        let counts = (period_ms * TICKS_PER_MS) / psc.divider();
        if counts == 0 {
            return Err(Error::TooShort);
        }
        if counts > 0x1000 {
            return Err(Error::TooLong);
        }
        // The counter reloads with RL and counts down through zero, so the
        // period is (RL + 1) * divider / LSI.
        let reload = (counts - 1) as u16;

        unsafe {
            // Unlock PSCR and RLDR for writing.
            write_volatile(CTLR, KEY_UNLOCK);

            // PSCR and RLDR are clocked by the LSI, not the bus: writes take
            // several LSI cycles to land and are dropped silently if the
            // corresponding busy flag is still set.
            wait_clear(PVU)?;
            write_volatile(PSCR, psc as u16);

            wait_clear(RVU)?;
            write_volatile(RLDR, reload & 0x0FFF);

            write_volatile(CTLR, KEY_FEED); // load the reload value
            write_volatile(CTLR, KEY_START); // start counting (enables LSI)
        }
        Ok(Self)
    }

    /// Reset the countdown. Call well inside the configured period.
    #[inline]
    pub fn feed(&mut self) {
        unsafe { write_volatile(CTLR, KEY_FEED) };
    }

    /// Nominal period actually achieved for a prescaler and reload value,
    /// in milliseconds. Rounding means this may differ from what was asked
    /// for — though LSI tolerance swamps the difference in practice.
    pub const fn actual_ms(psc: Prescaler, reload: u16) -> u32 {
        ((reload as u32 + 1) * psc.divider()) / TICKS_PER_MS
    }
}
