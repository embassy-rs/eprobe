##!/bin/bash

set -euxo pipefail

cd $(dirname $0)

rm -rf build
mkdir -p build

# bootloader
(cd bootloader; cargo build --release)
cp bootloader/target/thumbv7m-none-eabi/release/eprobe-bootloader build/bootloader.elf
llvm-objcopy -O binary build/bootloader.elf build/bootloader.bin

# bootloader loader
# it embeds the bootloader, so must be built after.
(cd bootloaderloader; cargo build --release)
cp bootloaderloader/target/thumbv7m-none-eabi/release/eprobe-bootloaderloader build/bootloaderloader.elf
llvm-objcopy -O binary build/bootloaderloader.elf build/bootloaderloader.bin

# application
(cd application; cargo build --release)
cp application/target/thumbv7m-none-eabi/release/eprobe build/application.elf
llvm-objcopy -O binary build/application.elf build/application.bin

#sudo dfu-util -d c0de:cafe,c0de:caff -D build/application.bin -R
