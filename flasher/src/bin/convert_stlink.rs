use std::time::Duration;

use aes::Aes128;
use cipher::{generic_array::GenericArray, BlockEncrypt, KeyInit};
use hex_literal::hex;
use log::{info, trace};
use nusb::{
    transfer::{Bulk, In, Out},
    Endpoint, MaybeFuture,
};

/// Convert array to/from big-endian format by swapping bytes in 4-byte chunks
fn byteswap(data: &mut [u8]) {
    for chunk in data.chunks_exact_mut(4) {
        let value = u32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        chunk.copy_from_slice(&value.to_be_bytes());
    }
}

/// AES ECB encryption with big-endian conversion
fn encrypt(key: &[u8; 16], data: &mut [u8]) {
    let mut key_be = *key;
    byteswap(&mut key_be);
    byteswap(data);

    let cipher = Aes128::new(GenericArray::from_slice(&key_be));
    for chunk in data.chunks_exact_mut(16) {
        cipher.encrypt_block(GenericArray::from_mut_slice(chunk));
    }

    byteswap(data);
}

fn main() {
    pretty_env_logger::init();

    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage: {} <firmware.bin>", args[0]);
        std::process::exit(1);
    }
    let firmware_path = &args[1];

    let di = nusb::list_devices()
        .wait()
        .unwrap()
        .find(|d| d.vendor_id() == 0x0483 && d.product_id() == 0x3748)
        .expect("device should be connected");

    info!("USB device info: {di:?}");

    let device = di.open().wait().unwrap();
    device.reset().wait().unwrap();
    let iface = device.claim_interface(0).wait().unwrap();
    let mut ep_out = iface.endpoint::<Bulk, Out>(0x02).unwrap();
    let mut ep_in = iface.endpoint::<Bulk, In>(0x81).unwrap();

    // Read device info (0xF180 command)
    let version = cmd(
        &mut ep_out,
        &mut ep_in,
        hex!("f1800000000000000000000000000000"),
    );
    let stlink_version = version[0] >> 4;
    let jtag_version = (version[0] & 0x0F) << 2 | (version[1] & 0xC0) >> 6;
    let swim_version = version[1] & 0x3F;
    let loader_version = (version[5] as u16) << 8 | version[4] as u16;
    info!(
        "Firmware version : V{}J{}S{}",
        stlink_version, jtag_version, swim_version
    );
    info!("Loader version : {}", loader_version);

    // Get current mode (0xF5 command)
    let state = cmd(
        &mut ep_out,
        &mut ep_in,
        hex!("f5000000000000000000000000000000"),
    );
    let mode = (state[0] as u16) << 8 | state[1] as u16;
    info!("Current mode : {}", mode);

    if mode != 1 {
        eprintln!(
            "ST-Link dongle is not in the correct mode. Please unplug and plug the dongle again."
        );
        std::process::exit(1);
    }

    // Get device ID (0xF308 command)
    let res = cmd(
        &mut ep_out,
        &mut ep_in,
        hex!("f3080000000000000000000000000000"),
    );

    // Extract ST-Link ID
    let mut id = [0u8; 12];
    id.copy_from_slice(&res[8..20]);
    info!("ST-Link ID : {:02X?}", id);

    // Calculate encryption key
    let mut firmware_key = [0u8; 16];
    firmware_key[0..4].copy_from_slice(&res[0..4]);
    firmware_key[4..16].copy_from_slice(&res[8..20]);
    let master_key = b"I am key, wawawa";
    encrypt(master_key, &mut firmware_key);
    info!("Firmware encryption key : {:02X?}", firmware_key);

    // Get initial status
    cmd(
        &mut ep_out,
        &mut ep_in,
        hex!("f3030000000006000000000000000000"),
    );

    // Load and flash firmware
    let data = std::fs::read(firmware_path).expect("Failed to read firmware file");
    info!(
        "Loaded firmware : {}, size : {} bytes",
        firmware_path,
        data.len()
    );

    let base_addr = 0x08004000;
    const CHUNK_SIZE: usize = 1024;
    for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
        let chunk_addr = base_addr + (i * CHUNK_SIZE) as u32;
        trace!("downloading at {:08x}", chunk_addr);
        erase(&mut ep_out, &mut ep_in, chunk_addr);
        set_address(&mut ep_out, &mut ep_in, chunk_addr);

        let mut buf = vec![0xFF; CHUNK_SIZE];
        buf[..chunk.len()].copy_from_slice(chunk);
        encrypt(&firmware_key, &mut buf);

        download(&mut ep_out, &mut ep_in, 2, buf);
        wait(&mut ep_out, &mut ep_in);
        print!(".");
        use std::io::Write;
        std::io::stdout().flush().unwrap();
    }
    println!();

    // Exit DFU - device will disconnect, so ignore errors
    bulk_out(&mut ep_out, &hex!("f3070000000006000000000000000000"));
    // Don't wait for response, device disconnects

    println!("done!");
}

fn set_address(ep_out: &mut Endpoint<Bulk, Out>, ep_in: &mut Endpoint<Bulk, In>, addr: u32) {
    let mut data = [0u8; 5];
    data[0] = 0x21;
    data[1..5].copy_from_slice(&addr.to_le_bytes());
    download(ep_out, ep_in, 0, data);
    wait(ep_out, ep_in);
}

fn erase(ep_out: &mut Endpoint<Bulk, Out>, ep_in: &mut Endpoint<Bulk, In>, addr: u32) {
    let mut data = [0u8; 5];
    data[0] = 0x41;
    data[1..5].copy_from_slice(&addr.to_le_bytes());
    download(ep_out, ep_in, 0, data);
    wait(ep_out, ep_in);
}

fn wait(ep_out: &mut Endpoint<Bulk, Out>, ep_in: &mut Endpoint<Bulk, In>) {
    loop {
        let status = cmd(ep_out, ep_in, hex!("f3030000000006000000000000000000"));
        if status[4] & 1 != 0 {
            break;
        }

        let timeout = status[1] as u32 | (status[2] as u32) << 8 | (status[3] as u32) << 16;
        std::thread::sleep(Duration::from_millis(timeout as _));
    }
}

fn download(
    ep_out: &mut Endpoint<Bulk, Out>,
    _ep_in: &mut Endpoint<Bulk, In>,
    block_num: u16,
    data: impl Into<Vec<u8>>,
) {
    let data = data.into();
    let mut checksum = 0u16;
    for &b in &data {
        checksum = checksum.wrapping_add(b as u16);
    }
    let mut header = [0u8; 16];
    header[0..2].copy_from_slice(&hex!("f301"));
    header[2..4].copy_from_slice(&block_num.to_le_bytes());
    header[4..6].copy_from_slice(&checksum.to_le_bytes());
    header[6..8].copy_from_slice(&(data.len() as u16).to_le_bytes());

    bulk_out(ep_out, header);
    bulk_out(ep_out, data);
}

fn cmd(
    ep_out: &mut Endpoint<Bulk, Out>,
    ep_in: &mut Endpoint<Bulk, In>,
    req: impl Into<Vec<u8>>,
) -> Vec<u8> {
    bulk_out(ep_out, req);
    bulk_in(ep_in)
}

fn bulk_out(ep_out: &mut Endpoint<Bulk, Out>, req: impl Into<Vec<u8>>) {
    let req = req.into();
    trace!("> {:02x?}", req);
    ep_out.submit(req.into());
    ep_out
        .wait_next_complete(Duration::from_millis(1000))
        .unwrap()
        .status
        .unwrap();
}

fn bulk_in(ep_in: &mut Endpoint<Bulk, In>) -> Vec<u8> {
    let buffer = ep_in.allocate(256);
    ep_in.submit(buffer);
    let completion = ep_in
        .wait_next_complete(Duration::from_millis(1000))
        .unwrap();
    completion.status.unwrap();
    let res = completion.buffer.to_vec();
    trace!("< {:02x?}", res);
    res
}
