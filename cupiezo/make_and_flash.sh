#!/bin/sh
BIN=$1
DISK=/dev/disk/by-id/usb-RPI_RP2_E0C912952D54-0:0-part1
cargo build --bin $BIN --release || exit 1
while true; do
  echo Waiting for rp2040
  while [ ! -e $DISK ]; do
    sleep 1
  done
  sudo umount /mnt
  sudo mount $DISK /mnt/ -o uid=1000,gid=1000 || exit 1
  elf2uf2-rs -d target/thumbv6m-none-eabi/release/$BIN && sudo umount /mnt
  sudo umount /mnt
  sleep 1;
done
