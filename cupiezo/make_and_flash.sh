#!/bin/sh
BIN=$1
set -x
cargo build --bin $BIN --release || exit 1
sudo umount /mnt
sudo mount /dev/disk/by-id/usb-RPI_RP2_E0C912952D54-0:0-part1 /mnt/ -o uid=1000,gid=1000 || exit 1
elf2uf2-rs -d target/thumbv6m-none-eabi/release/$BIN || exit 1
sudo umount /mnt || exit 1
