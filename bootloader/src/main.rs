#![no_std]
#![no_main]

mod fmt;

use cortex_m_rt::{exception, ExceptionFrame};
use embassy_stm32::flash::{Blocking, Flash};
use embassy_usb::msos::{self, windows_version};
#[cfg(not(feature = "defmt"))]
use panic_halt as _;
#[cfg(feature = "defmt")]
use {defmt_rtt as _, panic_probe as _};

use embassy_futures::block_on;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb, Config};
use embassy_usb::class::dfu::consts::{DfuAttributes, Status};
use embassy_usb::class::dfu::dfu_mode::{usb_dfu, DfuState, Handler};
use embassy_usb::Builder;
use static_cell::StaticCell;

#[exception]
unsafe fn HardFault(_: &ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => usb::InterruptHandler<peripherals::USB>;
});

extern "C" {
    static mut __persist: u32;
    static mut __app_start: u32;
    static __app_end: u32;
}
const PERSIST_BOOT_MAGIC: u32 = 0x93f2af30;

#[cortex_m_rt::entry]
fn main() -> ! {
    unsafe {
        let bootloader_requested = __persist == PERSIST_BOOT_MAGIC;
        let valid_app = __app_start != 0xFFFF_FFFF;

        // Immediately bootload the app if bootloader mode was not requested.
        if !bootloader_requested && valid_app {
            let p: *const u32 = &__app_start;
            (*cortex_m::peripheral::SCB::PTR).vtor.write(p as u32);
            cortex_m::asm::bootload(p);
        }

        // bootloader was requested. Clear the flag.
        __persist = 0;
    }

    let mut config = Config::default();
    config.rcc.hse = Some(Hertz(8_000_000));
    config.rcc.sys_ck = Some(Hertz(72_000_000));
    config.rcc.pclk1 = Some(Hertz(36_000_000));
    let p = embassy_stm32::init(config);

    info!("Hello World!");

    // Pinout
    // source: https://stm32world.com/images/b/bd/ST-Link_V2.1_Original_Schematics.png
    let usb_renum = p.PA15;
    let mut usb_dp = p.PA12;
    let usb_dm = p.PA11;
    let led = p.PA9;
    let t_jtck = p.PA5;
    let t_jtdo = p.PA6;
    let t_jtdi = p.PA7;
    let t_nrst = p.PB0;
    let t_jtck = p.PB13; // aka swclk
    let t_jtms = p.PB14; // aka swdio
    let t_swdio_in = p.PB12;
    let t_swo = p.PA10;
    let uart_tx = p.PA2;
    let uart_rx = p.PA3;

    // Seems not all stlink boards have the usb_renum transistor?
    // try forcing usb reenumeration with both USB_RENUM pin and pulling DP low directly.
    let usb_dp_renum = Output::new(&mut usb_dp, Level::Low, Speed::Low);
    let mut usb_renum = Output::new(usb_renum, Level::Low, Speed::Low);
    cortex_m::asm::delay(1_000_000);
    usb_renum.set_high();
    drop(usb_dp_renum);

    // Create embassy-usb Config.
    let mut config = embassy_usb::Config::new(0xc0de, 0xcaff);
    config.manufacturer = Some("Embassy");
    config.product = Some("Embassy Probe bootloader");
    config.serial_number = Some(embassy_stm32::uid::uid_hex());

    const CONTROL_BUF_SIZE: usize = 1024;

    // Create embassy-usb DeviceBuilder using the driver and config.
    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let driver = Driver::new(p.USB, Irqs, usb_dp, usb_dm);
    let mut builder = Builder::new(
        driver,
        config,
        &mut DEVICE_DESC.init([0; 256])[..],
        &mut CONFIG_DESC.init([0; 256])[..],
        &mut BOS_DESC.init([0; 256])[..],
        &mut MSOS_DESC.init([0; 256])[..],
        &mut CONTROL_BUF.init([0; CONTROL_BUF_SIZE])[..],
    );

    // Add the Microsoft OS Descriptor (MSOS/MOD) descriptor.
    // We tell Windows that this entire device is compatible with the "WINUSB" feature,
    // which causes it to use the built-in WinUSB driver automatically, which in turn
    // can be used by libusb/rusb/nusb without needing a custom driver or INF file.
    builder.msos_descriptor(windows_version::WIN8_1, 0);
    builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        // Random GUID, for Embassy Probe bootloader.
        msos::PropertyData::RegMultiSz(&["{B3E4F8E0-7460-4B2B-8F68-1E4178DD83B8}"]),
    ));

    let handler = DfuHandler {
        flash: Flash::new_blocking(p.FLASH),
        downloaded: false,
        erase: 0,
        write: 0,
    };

    static STATE: StaticCell<DfuState<DfuHandler>> = StaticCell::new();
    let attrs = DfuAttributes::CAN_DOWNLOAD
        | DfuAttributes::WILL_DETACH
        | DfuAttributes::MANIFESTATION_TOLERANT;
    let state = STATE.init(DfuState::new(handler, attrs));
    usb_dfu(&mut builder, state, CONTROL_BUF_SIZE);

    // Start USB.
    let mut usb = builder.build();
    block_on(usb.run())
}

const FLASH_BASE: u32 = 0x0800_0000;
const PAGE_SIZE: u32 = 0x400;

fn app_start_offset() -> u32 {
    let n = unsafe { &__app_start } as *const u32 as u32;
    n - FLASH_BASE
}

fn app_end_offset() -> u32 {
    let n = unsafe { &__app_end } as *const u32 as u32;
    n - FLASH_BASE
}

struct DfuHandler {
    flash: Flash<'static, Blocking>,
    downloaded: bool,
    erase: u32,
    write: u32,
}

impl Handler for DfuHandler {
    fn start(&mut self) {
        info!("start");
        self.erase = app_start_offset();
        self.write = app_start_offset();
        self.downloaded = false;
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Status> {
        info!("write {}", data.len());

        let next_write = self.write + data.len() as u32;
        if next_write > app_end_offset() {
            return Err(Status::ErrAddress);
        }

        while self.erase < next_write {
            info!("erasing {:08x}", self.erase);
            let res = self
                .flash
                .blocking_erase(self.erase, self.erase + PAGE_SIZE);
            if let Err(e) = res {
                warn!("failed to erase: {:?}", e);
                return Err(Status::ErrErase);
            }
            self.erase += PAGE_SIZE;
        }

        info!("writing {:08x}", self.write);
        let res = self.flash.blocking_write(self.write, data);
        if let Err(e) = res {
            warn!("failed to write: {:?}", e);
            return Err(Status::ErrWrite);
        }
        self.write = next_write;

        Ok(())
    }

    fn finish(&mut self) -> Result<(), Status> {
        info!("finish");
        self.downloaded = true;
        Ok(())
    }

    fn reset(&mut self) {
        info!("reset");
        if self.downloaded {
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}
