use std::fs::File;

use probe_rs::{
    config::TargetDescriptionSource,
    flashing::{DownloadOptions, ElfOptions, FlashLoader, Format},
    probe::{list::Lister, DebugProbe, Probe},
    MemoryInterface,
};

const FLASH_ACR: u64 = 0x40022000;
const FLASH_KEYR: u64 = 0x40022004;
const FLASH_OPTKEYR: u64 = 0x40022008;
const FLASH_SR: u64 = 0x4002200C;
const FLASH_CR: u64 = 0x40022010;
const FLASH_AR: u64 = 0x40022014;
const FLASH_OBR: u64 = 0x4002201C;
const FLASH_WRPR: u64 = 0x40022020;

/* FLASH_CR register bits */

const FLASH_CR_PG: u32 = 1 << 0;
const FLASH_CR_PER: u32 = 1 << 1;
const FLASH_CR_MER: u32 = 1 << 2;
const FLASH_CR_OPTPG: u32 = 1 << 4;
const FLASH_CR_OPTER: u32 = 1 << 5;
const FLASH_CR_STRT: u32 = 1 << 6;
const FLASH_CR_LOCK: u32 = 1 << 7;
const FLASH_CR_OPTWRE: u32 = 1 << 9;
const FLASH_CR_OBL_LAUNCH: u32 = 1 << 13;

/* FLASH_SR register bits */

const FLASH_SR_BSY: u32 = 1 << 0;
const FLASH_SR_PGERR: u32 = 1 << 2;
const FLASH_SR_WRPRTERR: u32 = 1 << 4;
const FLASH_SR_EOP: u32 = 1 << 5;

/* STM32_FLASH_OBR bit definitions (reading) */

const FLASH_OBR_ERROR: u32 = 1 << 0;
const FLASH_OBR_READOUT: u32 = 1 << 1;
const FLASH_OBR_RDWDGSW: u32 = 1 << 2;
const FLASH_OBR_RDRSTSTOP: u32 = 1 << 3;
const FLASH_OBR_RDRSTSTDBY: u32 = 1 << 4;
const FLASH_OBR_BFB2: u32 = 1 << 5;

const OB_RDP: u64 = 0x1FFFF800;
const OB_USER: u64 = 0x1FFFF802;
const OB_DATA0: u64 = 0x1FFFF804;
const OB_DATA1: u64 = 0x1FFFF806;
const OB_WRP0: u64 = 0x1FFFF808;
const OB_WRP1: u64 = 0x1FFFF80A;
const OB_WRP2: u64 = 0x1FFFF80C;
const OB_WRP3: u64 = 0x1FFFF80E;

const KEY1: u32 = 0x45670123;
const KEY2: u32 = 0xCDEF89AB;

fn main() {
    pretty_env_logger::init();

    let lister = Lister::new();

    let probes = lister.list_all();
    let probe_info = probes
        .into_iter()
        .find(|p| p.serial_number.as_deref() == Some("1C2703145216303030303032"))
        .unwrap();
    //let probe_info = probes[0];

    let probe = probe_info.open().unwrap();

    let mut sess = probe.attach("stm32f103c8tx", Default::default()).unwrap();
    let mut core = sess.core(0).unwrap();

    core.write_word_32(FLASH_KEYR, KEY1).unwrap();
    core.write_word_32(FLASH_KEYR, KEY2).unwrap();
    core.write_word_32(FLASH_OPTKEYR, KEY1).unwrap();
    core.write_word_32(FLASH_OPTKEYR, KEY2).unwrap();
    core.write_word_32(FLASH_CR, FLASH_CR_OPTER | FLASH_CR_OPTWRE)
        .unwrap();
    core.write_word_32(FLASH_CR, FLASH_CR_OPTER | FLASH_CR_OPTWRE | FLASH_CR_STRT)
        .unwrap();

    while core.read_word_32(FLASH_SR).unwrap() & FLASH_SR_BSY != 0 {}

    core.write_word_32(FLASH_CR, FLASH_CR_OPTPG | FLASH_CR_OPTWRE)
        .unwrap();

    core.write_word_16(OB_RDP, 0x00A5).unwrap();

    while core.read_word_32(FLASH_SR).unwrap() & FLASH_SR_BSY != 0 {}

    drop(core);

    let mmap = sess.target().memory_map.clone();
    let mut loader = FlashLoader::new(mmap, TargetDescriptionSource::BuiltIn);
    let mut bl = File::open("../../build/bootloader.elf").unwrap();
    let mut app = File::open("../../build/application.elf").unwrap();
    loader
        .load_image(&mut sess, &mut bl, Format::Elf(ElfOptions::default()), None)
        .unwrap();
    loader
        .load_image(
            &mut sess,
            &mut app,
            Format::Elf(ElfOptions::default()),
            None,
        )
        .unwrap();

    let mut opts = DownloadOptions::default();
    opts.verify = true;
    opts.do_chip_erase = true;
    loader.commit(&mut sess, opts).unwrap();

    sess.core(0).unwrap().reset().unwrap();
}
