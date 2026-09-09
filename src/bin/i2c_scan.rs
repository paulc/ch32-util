#![no_std]
#![no_main]

use ch32_util::sdi_println;

use ch32_hal as hal;
use hal::debug::SDIPrint;
use hal::delay::Delay;
use hal::i2c::I2c;
use hal::time::Hertz;

#[qingke_rt::entry]
fn main() -> ! {
    SDIPrint::enable();

    let config = hal::Config::default();
    let p = hal::init(config);
    let mut delay = Delay;

    sdi_println!(">> INIT");

    // Reset Status
    let rst = hal::pac::RCC.rstsckr().read();
    sdi_println!(">> RST: {:x}", rst.0);

    // Clear RST and DATA0 (for sdi_write)
    hal::pac::RCC.rstsckr().modify(|w| w.set_rmvf(true)); // clear for next time

    ch32_util::chip_info::chip_info();

    let sda = p.PC1;
    let scl = p.PC2;

    let mut i2c = I2c::new_blocking(p.I2C1, scl, sda, Hertz::hz(100_000), Default::default());

    loop {
        sdi_println!("Scan I2C bus: START");
        for addr in 1..=127 {
            let mut buf = [0u8; 1];
            if i2c.blocking_read(addr, &mut buf).is_ok() {
                sdi_println!(">> Found I2C device at address: 0x{:02x}", addr);
            }
        }
        sdi_println!("Scan I2C bus: DONE");
        delay.delay_ms(2000);
    }
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    ch32_util::sdi_write::sdi_write(b"!! PANIC !!\n");
    loop {}
}
