#![no_std]
#![no_main]

use cortex_m_rt::{exception, ExceptionFrame};
use embassy_stm32::{flash::Flash, time::Hertz, Config};
use panic_halt as _;

#[exception]
unsafe fn HardFault(_: &ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

extern "C" {
    static mut __persist: u32;
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hse = Some(Hse {
            freq: Hertz(8_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll = Some(Pll {
            src: PllSource::HSE,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL9,
        });
        config.rcc.sys = Sysclk::PLL1_P;
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
    }
    let p = embassy_stm32::init(config);

    // Copy bootloader
    const PAGE_SIZE: u32 = 0x400;
    let data = &include_bytes!("../../build/bootloader.bin");

    let mut flash = Flash::new_blocking(p.FLASH);
    for (i, chunk) in data.chunks(PAGE_SIZE as _).enumerate() {
        let i = i as u32;
        flash
            .blocking_erase(i * PAGE_SIZE, (i + 1) * PAGE_SIZE)
            .unwrap();
        flash.blocking_write(i * PAGE_SIZE, chunk).unwrap();
    }

    // Reset
    extern "C" {
        static mut __persist: u32;
    }
    const PERSIST_ERASE_MAGIC: u32 = 0xf342c1b0;

    unsafe { __persist = PERSIST_ERASE_MAGIC };
    cortex_m::peripheral::SCB::sys_reset();
}
