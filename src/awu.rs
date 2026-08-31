//! Minimal PWR / auto-wakeup (AWU) wrapper for the CH32V003.
//!
//! ch32-metapac defines `pwr_v00x` but does not instantiate PWR for the V003,
//! so this drives the registers directly. Addresses from section 2.4 of the
//! CH32V003 reference manual:
//!
//! ```text
//! R32_PWR_CTLR    0x40007000  Power control            reset 0x00000000
//! R32_PWR_CSR     0x40007004  Power control/status     reset 0x00000000
//! R32_PWR_AWUCSR  0x40007008  Auto-wakeup control      reset 0x00000000
//! R32_PWR_AWUWR   0x4000700C  Auto-wakeup window       reset 0x0000003F
//! R32_PWR_AWUPSC  0x40007010  Auto-wakeup prescaler    reset 0x00000000
//! ```
//!
//! Setup order matters:
//!
//! ```ignore
//! pwr_enable()?;              // APB1 clock — without this, PWR writes vanish
//! lsi_enable()?;              // AWU counter clock
//! awu_configure(psc, ms)?;
//! awu_enable();
//! standby();
//! ```
//!
//! The AWU counter is clocked by the LSI (nominally 128 kHz), an untrimmed RC
//! oscillator with wide tolerance. Treat all periods as approximate.
//!
//! IMPORTANT: the V003 does not stop the IWDG in standby. Sleeping longer than
//! the watchdog period gets you a watchdog reset rather than an AWU wake.

use core::ptr::{read_volatile, write_volatile};

// ---------------------------------------------------------------- PWR

const PWR_BASE: usize = 0x4000_7000;
const PWR_CTLR: *mut u32 = PWR_BASE as *mut u32;
const PWR_CSR: *const u32 = (PWR_BASE + 0x04) as *const u32;
const PWR_AWUCSR: *mut u32 = (PWR_BASE + 0x08) as *mut u32;
const PWR_AWUWR: *mut u32 = (PWR_BASE + 0x0C) as *mut u32;
const PWR_AWUPSC: *mut u32 = (PWR_BASE + 0x10) as *mut u32;

const CTLR_PDDS: u32 = 1 << 1; // 1 = standby, 0 = sleep
const AWUCSR_AWUEN: u32 = 1 << 1;

// ---------------------------------------------------------------- RCC

const RCC_APB1PCENR: *mut u32 = 0x4002_101C as *mut u32;
const APB1_PWREN: u32 = 1 << 28;

const RCC_RSTSCKR: *mut u32 = 0x4002_1024 as *mut u32;
const RSTSCKR_LSION: u32 = 1 << 0;
const RSTSCKR_LSIRDY: u32 = 1 << 1;

// ---------------------------------------------------------------- EXTI

const EXTI_BASE: usize = 0x4001_0400;
const EXTI_INTENR: *mut u32 = EXTI_BASE as *mut u32; // 0x00
const EXTI_EVENR: *mut u32 = (EXTI_BASE + 0x04) as *mut u32; // 0x04
const EXTI_RTENR: *mut u32 = (EXTI_BASE + 0x08) as *mut u32; // 0x08
const EXTI_FTENR: *mut u32 = (EXTI_BASE + 0x0C) as *mut u32; // 0x0C
const EXTI_INTFR: *mut u32 = (EXTI_BASE + 0x14) as *mut u32; // 0x14, write 1 to clear

/// The AWU is routed to the core as EXTI line 9.
const EXTI_AWU: u32 = 1 << 9;

// ---------------------------------------------------------------- PFIC

const PFIC_SCTLR: *mut u32 = 0xE000_ED10 as *mut u32;
const SCTLR_SLEEPDEEP: u32 = 1 << 2;
const SCTLR_WFITOWFE: u32 = 1 << 3;

const LSI_HZ: u32 = 128_000;
const LSI_SPIN_LIMIT: u32 = 100_000;

/// AWU counter prescaler. The encoding is irregular — not a power-of-two
/// exponent, and the top two values jump to /10240 and /61440.
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

/// Deliberately has no `Debug` impl: deriving one would pull `core::fmt` into
/// the binary via `Result::unwrap`. Match on this rather than unwrapping.
#[derive(Clone, Copy)]
pub enum Error {
    /// Interval is shorter than one count at this prescaler.
    TooShort,
    /// Interval needs more than 64 counts at this prescaler.
    TooLong,
    /// LSIRDY never came up.
    LsiTimeout,
    /// PWR registers did not accept writes — check the APB1 clock enable.
    PwrNotClocked,
}

/// Enable the PWR peripheral's APB1 clock.
///
/// Without this every write to PWR_CTLR/AWUCSR/AWUWR/AWUPSC is silently
/// discarded and reads back as zero. Call before anything else here.
pub fn pwr_enable() -> Result<(), Error> {
    unsafe {
        write_volatile(RCC_APB1PCENR, read_volatile(RCC_APB1PCENR) | APB1_PWREN);
        // AWUWR resets to 0x3F, so a zero read means the block is still dark.
        if read_volatile(PWR_AWUWR) & 0x3F == 0 {
            return Err(Error::PwrNotClocked);
        }
    }
    Ok(())
}

/// Enable the LSI. The AWU has no clock without it, and unlike the IWDG,
/// enabling the AWU does not start the LSI implicitly.
pub fn lsi_enable() -> Result<(), Error> {
    unsafe {
        write_volatile(RCC_RSTSCKR, read_volatile(RCC_RSTSCKR) | RSTSCKR_LSION);
        let mut spins = 0u32;
        while read_volatile(RCC_RSTSCKR) & RSTSCKR_LSIRDY == 0 {
            spins += 1;
            if spins > LSI_SPIN_LIMIT {
                return Err(Error::LsiTimeout);
            }
        }
    }
    Ok(())
}

/// Configure the auto-wakeup counter. The window value is compared against an
/// up-counter; a wake fires when they match.
///
/// With literal arguments the arithmetic constant-folds; a runtime
/// `interval_ms` pulls in a software divide (`__udivsi3`) on this core.
pub fn awu_configure(psc: Prescaler, interval_ms: u32) -> Result<(), Error> {
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
        write_volatile(PWR_AWUPSC, psc as u32);
        write_volatile(PWR_AWUWR, (counts - 1) & 0x3F);
    }
    Ok(())
}

/// Nominal interval for a prescaler and window value, in milliseconds.
pub const fn actual_ms(psc: Prescaler, window: u8) -> u32 {
    ((window as u32 + 1) * psc.divider()) / (LSI_HZ / 1000)
}

/// Route the AWU to the core via EXTI line 9 and enable the counter.
///
/// Both edges are enabled: which one the AWU presents is not stated clearly in
/// the manual, and enabling both costs nothing.
pub fn awu_enable() {
    unsafe {
        write_volatile(EXTI_EVENR, read_volatile(EXTI_EVENR) | EXTI_AWU);
        write_volatile(EXTI_INTENR, read_volatile(EXTI_INTENR) | EXTI_AWU);
        write_volatile(EXTI_RTENR, read_volatile(EXTI_RTENR) | EXTI_AWU);
        write_volatile(EXTI_FTENR, read_volatile(EXTI_FTENR) | EXTI_AWU);
        write_volatile(PWR_AWUCSR, AWUCSR_AWUEN);
    }
}

pub fn awu_disable() {
    unsafe {
        write_volatile(PWR_AWUCSR, 0);
        write_volatile(EXTI_EVENR, read_volatile(EXTI_EVENR) & !EXTI_AWU);
        write_volatile(EXTI_INTENR, read_volatile(EXTI_INTENR) & !EXTI_AWU);
    }
}

/// Clear a pending AWU event on EXTI line 9.
#[inline]
pub fn awu_clear_pending() {
    unsafe { write_volatile(EXTI_INTFR, EXTI_AWU) };
}

/// Restart the AWU counter from zero and clear any pending event.
///
/// The AWU is a free-running up-counter compared against the window value;
/// entering standby does not reset it. Without this, the second and every
/// subsequent sleep returns immediately because the counter is already at or
/// past the window. Cycling AWUEN holds the counter in reset and restarts it.
#[inline]
pub fn awu_restart() {
    unsafe {
        write_volatile(PWR_AWUCSR, 0);
        // Rewriting the window reloads the comparator on some revisions;
        // harmless if it doesn't.
        let wr = read_volatile(PWR_AWUWR) & 0x3F;
        write_volatile(PWR_AWUWR, wr);
        write_volatile(PWR_AWUCSR, AWUCSR_AWUEN);
    }
    awu_clear_pending();
}

/// Enter sleep: core clock stops, peripherals keep running, any enabled
/// interrupt or event wakes it. Execution resumes after the `wfi`.
///
/// NOTE: under Embassy this wakes on the next time-driver tick or task alarm,
/// not on the AWU — every peripheral interrupt is still live in sleep mode.
/// Use `standby` if you want the AWU to define the wake interval.
#[inline]
pub fn sleep() {
    awu_restart();
    unsafe {
        write_volatile(PWR_CTLR, read_volatile(PWR_CTLR) & !CTLR_PDDS);
        write_volatile(
            PFIC_SCTLR,
            (read_volatile(PFIC_SCTLR) & !SCTLR_SLEEPDEEP) | SCTLR_WFITOWFE,
        );
        core::arch::asm!("wfi");
    }
    awu_clear_pending();
}

/// Enter standby. Much lower current, but most of the chip is powered down.
/// Call `pwr_enable`, `lsi_enable`, `awu_configure` and `awu_enable` first.
const SCTLR_SEVONPEND: u32 = 1 << 4;
const SCTLR_SETEVENT: u32 = 1 << 5;
/// Interrupt pending reset register, write 1 to clear. TIM2 is IRQ 38 -> word 1, bit 6.
const PFIC_IPRR0: *mut u32 = 0xE000_E280 as *mut u32;

#[inline]
pub fn standby() {
    awu_restart();
    unsafe {
        write_volatile(PWR_CTLR, read_volatile(PWR_CTLR) | CTLR_PDDS);

        let saved = read_volatile(PFIC_SCTLR);
        write_volatile(
            PFIC_SCTLR,
            (saved | SCTLR_SLEEPDEEP | SCTLR_WFITOWFE) & !SCTLR_SEVONPEND,
        );

        // Drop a pending TIM2 so an enabled+pending irq can't wake the second wfe.
        write_volatile(PFIC_IPRR0.add(1), 1 << (38 - 32));

        // qingke-rt leaves WFITOWFE set, so `wfi` runs as `wfe` and consumes a
        // latched event rather than sleeping. ch32-hal's init() sets SEVONPEND,
        // and the ~1 Hz TIM2 time driver latches an event long before we get
        // here; clearing SEVONPEND above does not un-latch it. So: force the
        // latch set, burn it on the first wfe, sleep on the second.
        write_volatile(PFIC_SCTLR, read_volatile(PFIC_SCTLR) | SCTLR_SETEVENT);
        core::arch::asm!("wfi"); // returns immediately, clears the latch
        core::arch::asm!("wfi"); // actually enters standby

        write_volatile(PFIC_SCTLR, saved);
        write_volatile(PWR_CTLR, read_volatile(PWR_CTLR) & !CTLR_PDDS);
    }
    awu_clear_pending();
}

// ---------------------------------------------------------------- readback

/// Snapshot of every register involved in the AWU wake path.
///
/// No `Debug` impl by design — print the fields with your own hex formatter.
#[derive(Clone, Copy)]
pub struct Regs {
    pub pwr_ctlr: u32,
    pub pwr_csr: u32,
    pub awucsr: u32,
    pub awuwr: u32,
    pub awupsc: u32,
    pub rcc_apb1pcenr: u32,
    pub rcc_rstsckr: u32,
    pub exti_intenr: u32,
    pub exti_evenr: u32,
    pub exti_rtenr: u32,
    pub exti_ftenr: u32,
    pub exti_intfr: u32,
    pub pfic_sctlr: u32,
}

/// Read back the full wake path. Call immediately before sleeping.
pub fn read_regs() -> Regs {
    unsafe {
        Regs {
            pwr_ctlr: read_volatile(PWR_CTLR),
            pwr_csr: read_volatile(PWR_CSR),
            awucsr: read_volatile(PWR_AWUCSR),
            awuwr: read_volatile(PWR_AWUWR),
            awupsc: read_volatile(PWR_AWUPSC),
            rcc_apb1pcenr: read_volatile(RCC_APB1PCENR),
            rcc_rstsckr: read_volatile(RCC_RSTSCKR),
            exti_intenr: read_volatile(EXTI_INTENR),
            exti_evenr: read_volatile(EXTI_EVENR),
            exti_rtenr: read_volatile(EXTI_RTENR),
            exti_ftenr: read_volatile(EXTI_FTENR),
            exti_intfr: read_volatile(EXTI_INTFR),
            pfic_sctlr: read_volatile(PFIC_SCTLR),
        }
    }
}
