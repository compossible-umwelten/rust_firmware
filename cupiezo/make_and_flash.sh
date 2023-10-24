#!/bin/sh
# this file waits endlessly for the rp2040 to be plugged in and
# will try to mount it automatically when it is boot mode.
# and will flash the firmware, when done, it will unmount the device.

# you may want to change this DISK env var according to you device.
# DON'T use /dev/sdX syntax because that is not consistent. and you MAY accidentally
# format your Data and flash it.

BIN=$1
DISK=/dev/disk/by-id/usb-RPI_RP2_E0C912952D54-0:0-part1
while true; do
  echo Waiting for rp2040
  while [ ! -e $DISK ]; do
    sudo true # prevent password ask timeout
    sleep 1
  done
  sudo umount /mnt
  sudo mount $DISK /mnt/ -o uid=1000,gid=1000 || exit 1
  cargo build --release --bin $BIN
  elf2uf2-rs -d target/thumbv6m-none-eabi/release/$BIN && sudo umount /mnt
  sudo umount /mnt
  sleep 1;
done
