use std::time::Duration;

use aes::Aes128;
use cipher::{generic_array::GenericArray, BlockEncrypt, KeyInit};
use futures_lite::future::block_on;
use hex_literal::hex;
use log::{info, trace};
use nusb::{transfer::RequestBuffer, Interface};

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
    let di = nusb::list_devices()
        .unwrap()
        .find(|d| d.vendor_id() == 0x0483 && d.product_id() == 0x3748)
        .expect("device should be connected");

    info!("USB device info: {di:?}");

    let device = di.open().unwrap();
    device.reset().unwrap();
    let iface = &mut device.claim_interface(0).unwrap();

    cmd(iface, hex!("f1800000000000000000000000000000"));

    let state = cmd(iface, hex!("f5000000000000000000000000000000"));
    if state[0] != 0 {
        cmd(iface, hex!("f901"));
    }

    // Calculate encryption key
    let res = cmd(iface, hex!("f3080000000000000000000000000000"));
    let mut firmware_key = [0u8; 16];
    firmware_key[0..4].copy_from_slice(&res[0..4]);
    firmware_key[4..16].copy_from_slice(&res[8..20]);
    let master_key = b"I am key, wawawa";
    encrypt(master_key, &mut firmware_key);
    trace!("Firmware key: {:02x?}", firmware_key);

    cmd(iface, hex!("f3030000000006000000000000000000"));

    let base_addr = 0x08004000;
    let data = &include_bytes!("../../../build/bootloaderloader.bin");
    const CHUNK_SIZE: usize = 1024;
    for (i, chunk) in data.chunks(CHUNK_SIZE).enumerate() {
        let chunk_addr = base_addr + (i * CHUNK_SIZE) as u32;
        trace!("downloading at {:08x}", chunk_addr);
        erase(iface, chunk_addr);
        set_address(iface, chunk_addr);

        let mut buf = vec![0xFF; CHUNK_SIZE];
        buf[..chunk.len()].copy_from_slice(chunk);
        encrypt(&firmware_key, &mut buf);

        download(iface, 2, buf);
        wait(iface);
    }

    // Exit DFU
    cmd(iface, hex!("f3070000000006000000000000000000"));

    println!("done!");
}

fn set_address(iface: &mut Interface, addr: u32) {
    let mut data = [0u8; 5];
    data[0] = 0x21;
    data[1..5].copy_from_slice(&addr.to_le_bytes());
    download(iface, 0, data);
    wait(iface);
}

fn erase(iface: &mut Interface, addr: u32) {
    let mut data = [0u8; 5];
    data[0] = 0x41;
    data[1..5].copy_from_slice(&addr.to_le_bytes());
    download(iface, 0, data);
    wait(iface);
}

fn wait(iface: &mut Interface) {
    loop {
        let status = cmd(iface, hex!("f3030000000006000000000000000000"));
        if status[4] & 1 != 0 {
            break;
        }

        let timeout = status[1] as u32 | (status[2] as u32) << 8 | (status[3] as u32) << 16;
        std::thread::sleep(Duration::from_millis(timeout as _));
    }
}

fn download(iface: &mut Interface, block_num: u16, data: impl Into<Vec<u8>>) {
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

    bulk_out(iface, header);
    bulk_out(iface, data);
}

fn cmd(iface: &mut Interface, req: impl Into<Vec<u8>>) -> Vec<u8> {
    bulk_out(iface, req);
    bulk_in(iface)
}

fn bulk_out(iface: &mut Interface, req: impl Into<Vec<u8>>) {
    let req = req.into();
    trace!("> {:02x?}", req);
    block_on(iface.bulk_out(0x02, req)).into_result().unwrap();
}

fn bulk_in(iface: &mut Interface) -> Vec<u8> {
    let res: Vec<u8> = block_on(iface.bulk_in(0x81, RequestBuffer::new(256)))
        .into_result()
        .unwrap();
    trace!("< {:02x?}", res);
    res
}
