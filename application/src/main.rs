#![no_std]
#![no_main]

use cortex_m_rt::{exception, ExceptionFrame};
use dap_rs::dap::{Dap, DapLeds, DapVersion};
use dap_rs::swd::{self, APnDP, DPRegister};
use dap_rs::swj::Dependencies;
use dap_rs::swo::Swo;
use defmt::{todo, *};
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Flex, Level, Output, Pull, Speed};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, peripherals, usb, Config};
use embassy_time::{Delay, Duration, Timer};
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
    static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESC: StaticCell<[u8; 196]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 128]> = StaticCell::new();
    let driver = Driver::new(p.USB, Irqs, usb_dp, usb_dm);
    let mut builder = Builder::new(
        driver,
        config,
        &mut DEVICE_DESC.init([0; 256])[..],
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
    let mut read_ep = altsetting.endpoint_bulk_out(64);
    let mut write_ep = altsetting.endpoint_bulk_in(64);
    drop(function);

    static CONTROL_HANDLER: StaticCell<Control> = StaticCell::new();
    builder.handler(CONTROL_HANDLER.init(Control {
        iface_string: iface_string,
    }));

    static STATE: StaticCell<DfuState<DfuHandler>> = StaticCell::new();
    let attrs = DfuAttributes::CAN_DOWNLOAD
        | DfuAttributes::WILL_DETACH
        | DfuAttributes::MANIFESTATION_TOLERANT;
    let state = STATE.init(DfuState::new(DfuHandler { spawner }, attrs));
    usb_dfu(&mut builder, state);

    // Start USB.
    let usb = builder.build();
    unwrap!(spawner.spawn(usb_task(usb)));

    // Process DAP commands in a loop.
    let deps = Deps::new(Flex::new(t_jtck), Flex::new(t_jtms));

    let mut dap = Dap::new(deps, Leds, Delay, None::<NoSwo>, "Embassy CMSIS-DAP");

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
        let resp_len = dap.process_command(&req[..req_len], &mut resp, DapVersion::V2);

        if let Err(e) = write_packet(&mut write_ep, &resp[..resp_len]).await {
            warn!("failed to write to USB: {:?}", e);
            continue;
        }
    }
}

async fn read_packet(ep: &mut impl EndpointOut, buf: &mut [u8]) -> Result<usize, EndpointError> {
    let mut n = 0;

    loop {
        let i = ep.read(&mut buf[n..]).await?;
        n += i;
        if i < 64 {
            return Ok(n);
        }
    }
}

async fn write_packet(ep: &mut impl EndpointIn, buf: &[u8]) -> Result<(), EndpointError> {
    for chunk in buf.chunks(64) {
        ep.write(chunk).await?;
    }
    if buf.len() % 64 == 0 {
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

pub type SwclkPin = peripherals::PB13;
pub type SwdioPin = peripherals::PB14;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum Dir {
    Write = 0,
    Read = 1,
}

struct Deps {
    ck: Flex<'static, SwclkPin>,
    io: Flex<'static, SwdioPin>,
}

impl Deps {
    pub fn new(mut ck: Flex<'static, SwclkPin>, mut io: Flex<'static, SwdioPin>) -> Self {
        //io.set_pull(Pull::Up);
        io.set_high();
        io.set_as_output(Speed::VeryHigh);

        //ck.set_pull(Pull::None);
        ck.set_as_output(Speed::VeryHigh);

        Self { ck, io }
    }

    fn req(&mut self, port: APnDP, dir: Dir, addr: DPRegister) {
        let req = (port as u32) | (dir as u32) << 1 | (addr as u32) << 2;
        let parity = req.count_ones() % 2;
        self.shift_out(0b10000001 | req << 1 | parity << 5, 8);
    }

    pub fn read(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.req(port, Dir::Read, addr);

        self.shift_in(1); // turnaround

        let ack = self.shift_in(3);
        match ack {
            0b001 => {} // ok
            0b010 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckWait);
            }
            0b100 => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckFault);
            }
            _ => {
                self.shift_in(1); // turnaround
                return Err(swd::Error::AckUnknown(ack as u8));
            }
        }

        let data = self.shift_in(32);
        let parity = self.shift_in(1);
        if parity != data.count_ones() % 2 {
            return Err(swd::Error::BadParity);
        }

        self.shift_in(1); // turnaround

        Ok(data)
    }

    pub fn write(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.req(port, Dir::Write, addr);

        self.shift_in(1); // turnaround

        let ack = self.shift_in(3);
        self.shift_in(1); // turnaround
        match ack {
            0b001 => {} // ok
            0b010 => return Err(swd::Error::AckWait),
            0b100 => return Err(swd::Error::AckFault),
            _ => return Err(swd::Error::AckUnknown(ack as _)),
        }

        self.shift_out(data, 32);
        self.shift_out(data.count_ones() % 2, 1);

        Ok(())
    }

    fn shift_out(&mut self, val: u32, n: u32) {
        self.io.set_as_output(Speed::VeryHigh);
        for i in 0..n {
            self.io.set_level(Level::from(val & (1 << i) != 0));
            wait();
            self.ck.set_high();
            wait();
            self.ck.set_low();
        }
    }

    fn shift_in(&mut self, n: u32) -> u32 {
        self.io.set_as_input(Pull::None);
        let mut val = 0;
        for i in 0..n {
            val |= (self.io.is_high() as u32) << i;
            wait();
            self.ck.set_high();
            wait();
            self.ck.set_low();
        }
        val
    }
}

fn wait() {
    cortex_m::asm::delay(10);
}

impl Dependencies<Deps, Deps> for Deps {
    fn high_impedance_mode(&mut self) {
        // todo
    }

    fn process_swj_clock(&mut self, max_frequency: u32) -> bool {
        todo!()
    }

    fn process_swj_pins(
        &mut self,
        output: dap_rs::swj::Pins,
        mask: dap_rs::swj::Pins,
        wait_us: u32,
    ) -> dap_rs::swj::Pins {
        todo!()
    }

    fn process_swj_sequence(&mut self, data: &[u8], mut nbits: usize) {
        for &b in data {
            if nbits == 0 {
                break;
            }
            self.shift_out(b as u32, nbits.min(8) as u32);
            nbits -= 8;
        }
    }
}

impl dap_rs::swd::Swd<Deps> for Deps {
    const AVAILABLE: bool = true;

    fn read_inner(&mut self, port: APnDP, addr: DPRegister) -> swd::Result<u32> {
        self.read(port, addr)
    }

    fn write_inner(&mut self, port: APnDP, addr: DPRegister, data: u32) -> swd::Result<()> {
        self.write(port, addr, data)
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        // todo
        true
    }
}

impl dap_rs::jtag::Jtag<Deps> for Deps {
    const AVAILABLE: bool = false;

    fn sequences(&mut self, data: &[u8], rxbuf: &mut [u8]) -> u32 {
        todo!()
    }

    fn set_clock(&mut self, max_frequency: u32) -> bool {
        todo!()
    }
}

struct Leds;

impl DapLeds for Leds {
    fn react_to_host_status(&mut self, host_status: dap_rs::dap::HostStatus) {
        // TODO
    }
}

struct NoSwo;

impl Swo for NoSwo {
    fn set_transport(&mut self, transport: dap_rs::swo::SwoTransport) {
        todo!()
    }

    fn set_mode(&mut self, mode: dap_rs::swo::SwoMode) {
        todo!()
    }

    fn set_baudrate(&mut self, baudrate: u32) -> u32 {
        todo!()
    }

    fn set_control(&mut self, control: dap_rs::swo::SwoControl) {
        todo!()
    }

    fn polling_data(&mut self, buf: &mut [u8]) -> u32 {
        todo!()
    }

    fn streaming_data(&mut self) {
        todo!()
    }

    fn is_active(&self) -> bool {
        todo!()
    }

    fn bytes_available(&self) -> u32 {
        todo!()
    }

    fn buffer_size(&self) -> u32 {
        todo!()
    }

    fn support(&self) -> dap_rs::swo::SwoSupport {
        todo!()
    }

    fn status(&mut self) -> dap_rs::swo::SwoStatus {
        todo!()
    }
}

struct DfuHandler {
    spawner: Spawner,
}

impl embassy_usb::class::dfu::app_mode::Handler for DfuHandler {
    fn detach(&mut self) {
        self.spawner.spawn(reboot_task()).ok();
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
