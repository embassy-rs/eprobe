#![no_std]
#![no_main]

use cortex_m::interrupt::InterruptNumber;
use cortex_m_rt::{exception, ExceptionFrame};
use dap_rs::dap::{DapLeds, DelayNs};
use dap_rs::driver::bitbang::{
    BitbangAdapter, DelayCycles, InputOutputPin, JtagAdapter, SwdAdapter,
};
use dap_rs::jtag::TapConfig;
use dap_rs::swo::NoSwo;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Level, Output, Pull, Speed};
use embassy_stm32::pac;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb, Config};
use embassy_time::{Duration, Timer};
use embassy_usb::class::dfu::app_mode::{usb_dfu, DfuState};
use embassy_usb::class::dfu::consts::DfuAttributes;
use embassy_usb::driver::{Endpoint, EndpointError, EndpointIn, EndpointOut};
use embassy_usb::msos::{self, windows_version};
use embassy_usb::types::StringIndex;
use embassy_usb::{Builder, Handler, UsbDevice};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[exception]
unsafe fn HardFault(_: &ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset()
}

bind_interrupts!(struct Irqs {
    USB_LP_CAN1_RX0 => usb::InterruptHandler<peripherals::USB>;
});

type MyDriver = Driver<'static, peripherals::USB>;

#[embassy_executor::task]
async fn usb_task(mut device: UsbDevice<'static, MyDriver>) {
    device.run().await;
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    #[derive(Copy, Clone)]
    struct Irq(u16);
    unsafe impl InterruptNumber for Irq {
        fn number(self) -> u16 {
            self.0
        }
    }
    for i in 0..256 {
        cortex_m::peripheral::NVIC::mask(Irq(i));
    }

    unsafe { (*cortex_m::peripheral::SCB::PTR).vtor.write(0x4000) }

    // make sure usb renum works even if doing a warm boot (eg from the ST bootloader)
    pac::RCC.apb1rstr().modify(|w| {
        w.set_usbrst(true);
    });
    pac::RCC.apb2rstr().modify(|w| {
        w.set_gpioarst(true);
        w.set_gpiobrst(true);
        w.set_gpiocrst(true);
        w.set_gpiodrst(true);
        w.set_gpioerst(true);
        w.set_gpiofrst(true);
        w.set_afiorst(true);
    });
    pac::RCC.apb2rstr().modify(|w| {
        w.set_gpioarst(false);
        w.set_gpiobrst(false);
        w.set_gpiocrst(false);
        w.set_gpiodrst(false);
        w.set_gpioerst(false);
        w.set_gpiofrst(false);
        w.set_afiorst(false);
    });

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

    info!("Hello World!");

    // Pinout
    // source: https://stm32world.com/images/b/bd/ST-Link_V2.1_Original_Schematics.png
    let usb_renum = p.PA15;
    let mut usb_dp = p.PA12;
    let usb_dm = p.PA11;
    let _led = p.PA9;
    let _t_jtck_a5 = p.PA5;
    let t_jtdo = p.PA6; // unused for SWD, used for JTAG TDO
    let t_jtdi = p.PA7; // unused for SWD, used for JTAG TDI
    let t_nrst = p.PB0;
    let t_swclk = p.PB13;
    let t_swdio = p.PB14;
    let _t_swdio_in = p.PB12;
    let _t_swo = p.PA10;
    let _uart_tx = p.PA2;
    let _uart_rx = p.PA3;

    // Seems not all stlink boards have the usb_renum transistor?
    // try forcing usb reenumeration with both USB_RENUM pin and pulling DP low directly.
    let usb_dp_renum = Output::new(usb_dp.reborrow(), Level::Low, Speed::Low);
    let mut usb_renum = Output::new(usb_renum, Level::Low, Speed::Low);
    Timer::after_millis(10).await;
    usb_renum.set_high();
    drop(usb_dp_renum);

    // Create embassy-usb Config.
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("Embassy Probe CMSIS-DAP");
    config.serial_number = Some(embassy_stm32::uid::uid_hex());
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 196]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let driver = Driver::new(p.USB, Irqs, usb_dp, usb_dm);
    let mut builder = Builder::new(
        driver,
        config,
        &mut CONFIG_DESC.init([0; 256])[..],
        &mut BOS_DESC.init([0; 256])[..],
        &mut MSOS_DESC.init([0; 196])[..],
        &mut CONTROL_BUF.init([0; 128])[..],
    );

    builder.msos_descriptor(windows_version::WIN8_1, 0);

    let iface_string = builder.string();
    let mut function = builder.function(0xFF, 0, 0);
    function.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
    function.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
        "DeviceInterfaceGUIDs",
        // CMSIS-DAP standard GUID, from https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__ConfigUSB__gr.html
        msos::PropertyData::RegMultiSz(&["{CDB3B5AD-293B-4663-AA36-1AAE46463776}"]),
    ));
    let mut interface = function.interface();
    let mut altsetting = interface.alt_setting(0xFF, 0, 0, Some(iface_string));
    let mut read_ep = altsetting.endpoint_bulk_out(None, 64);
    let mut write_ep = altsetting.endpoint_bulk_in(None, 64);
    drop(function);

    static CONTROL_HANDLER: StaticCell<Control> = StaticCell::new();
    builder.handler(CONTROL_HANDLER.init(Control {
        iface_string: iface_string,
    }));

    static STATE: StaticCell<DfuState<DfuHandler>> = StaticCell::new();
    let attrs = DfuAttributes::CAN_DOWNLOAD
        | DfuAttributes::WILL_DETACH
        | DfuAttributes::MANIFESTATION_TOLERANT;
    let state = STATE.init(DfuState::new(
        DfuHandler { spawner },
        attrs,
        Duration::from_millis(1000),
    ));
    usb_dfu(&mut builder, state, |_| {});

    // Start USB.
    let usb = builder.build();
    spawner.spawn(usb_task(usb).unwrap());

    // Process DAP commands in a loop.
    // Create pin wrapper for bitbang adapter
    let nreset = Pin(Flex::new(t_nrst));
    let tdi = Pin(Flex::new(t_jtdi));
    let tms_swdio = Pin(Flex::new(t_swdio));
    let tck_swclk = Pin(Flex::new(t_swclk));
    let tdo = Pin(Flex::new(t_jtdo));

    static SCAN_CHAIN: StaticCell<[TapConfig; 8]> = StaticCell::new();
    let scan_chain = SCAN_CHAIN.init([TapConfig::INIT; 8]);

    let deps = BitbangAdapter::new(
        nreset, tdi, tms_swdio, tck_swclk, tdo, CycleDelay, scan_chain,
    );

    let mut dap = dap_rs::dap::Dap::<_, _, _, JtagAdapter<_, _>, SwdAdapter<_, _>, _>::new(
        deps,
        Leds,
        CycleDelay,
        NoSwo,
        "Embassy CMSIS-DAP",
        1024,
    );

    let mut req = [0u8; 1024];
    let mut resp = [0u8; 1024];
    loop {
        read_ep.wait_enabled().await;

        let req_len = match read_packet(&mut read_ep, &mut req).await {
            Ok(n) => n,
            Err(e) => {
                warn!("failed to read from USB: {:?}", e);
                continue;
            }
        };
        let resp_len = dap.process_command(&req[..req_len], &mut resp);

        if let Err(e) = write_packet(&mut write_ep, &resp[..resp_len]).await {
            warn!("failed to write to USB: {:?}", e);
            continue;
        }
    }
}

const USB_PACKET_SIZE: usize = 64;

async fn read_packet(ep: &mut impl EndpointOut, buf: &mut [u8]) -> Result<usize, EndpointError> {
    let mut n = 0;

    loop {
        let i = ep.read(&mut buf[n..]).await?;
        n += i;
        if i < USB_PACKET_SIZE {
            return Ok(n);
        }
    }
}

async fn write_packet(ep: &mut impl EndpointIn, buf: &[u8]) -> Result<(), EndpointError> {
    for chunk in buf.chunks(USB_PACKET_SIZE) {
        ep.write(chunk).await?;
    }
    if buf.len() % USB_PACKET_SIZE == 0 {
        ep.write(&[]).await?;
    }
    Ok(())
}

struct Control {
    iface_string: StringIndex,
}

impl Handler for Control {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        if index == self.iface_string {
            Some("CMSIS-DAP v2 Interface")
        } else {
            warn!("unknown string index requested");
            None
        }
    }
}

struct Pin(Flex<'static>);

impl InputOutputPin for Pin {
    fn set_as_output(&mut self) {
        self.0.set_as_output(Speed::VeryHigh);
    }

    fn set_high(&mut self, high: bool) {
        if high {
            self.0.set_high();
        } else {
            self.0.set_low();
        }
    }

    fn set_as_input(&mut self) {
        self.0.set_as_input(Pull::None);
    }

    fn is_high(&mut self) -> bool {
        self.0.is_high()
    }
}

const CPU_CLOCK: u32 = 72_000_000;

#[derive(Clone, Copy)]
struct CycleDelay;

impl DelayNs for CycleDelay {
    fn delay_ns(&mut self, ns: u32) {
        let cycles = (ns as u64 * CPU_CLOCK as u64 / 1_000_000_000) as u32;
        cortex_m::asm::delay(cycles);
    }
}

impl DelayCycles for CycleDelay {
    fn cpu_clock(&self) -> u32 {
        CPU_CLOCK
    }

    fn delay_cycles(&mut self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

struct Leds;

impl DapLeds for Leds {
    fn react_to_host_status(&mut self, _host_status: dap_rs::dap::HostStatus) {
        // TODO
    }
}

struct DfuHandler {
    spawner: Spawner,
}

impl embassy_usb::class::dfu::app_mode::Handler for DfuHandler {
    fn enter_dfu(&mut self) {
        self.spawner.spawn(reboot_task().unwrap());
    }
}

#[embassy_executor::task]
async fn reboot_task() {
    Timer::after(Duration::from_millis(500)).await;

    extern "C" {
        static mut __persist: u32;
    }
    const PERSIST_BOOT_MAGIC: u32 = 0x93f2af30;

    unsafe { __persist = PERSIST_BOOT_MAGIC };
    cortex_m::peripheral::SCB::sys_reset();
}
