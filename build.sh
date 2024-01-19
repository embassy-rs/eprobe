##!/bin/bash

set -euxo pipefail

cd $(dirname $0)

(cd bootloader; cargo build --release)
(cd application; cargo build --release)
cp bootloader/target/thumbv7m-none-eabi/release/eprobe-bootloader flasher/bootloader.elf
cp application/target/thumbv7m-none-eabi/release/eprobe flasher/application.elf
llvm-objcopy -O binary flasher/application.elf flasher/application.bin

#sudo dfu-util -d c0de:cafe,c0de:caff -D application.bin -R
