# Usage

simple program to test [stm32 nucleo-g071 board](https://www.st.com/en/evaluation-tools/nucleo-g071rb.html)

# Install arm gcc cross compiler
  [Download gcc ver 8-2019-q3-update for linux!](https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2)

```
  download arm gcc toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
  recommend gcc-8 or gcc-10, don't use gcc-9
  put arm-none-eabi-gcc in PATH
```

# Install stlink-utils

```
sudo apt install stlink-tools
```

# Build

```
./waf clean
./waf configure build
```

# Flash

Use stlink-tools to flash nucleo board.

```
./waf flash
[1/1] Processing build/st.230602.bin
st-flash 1.7.0
Mass erasing
st-flash 1.7.0
file st.230602.bin md5 checksum: 56eeed2b65acec5a4c800779e953e, stlink checksum: 0x00435fc1
 13/ 14 pages written

2023-06-02T14:35:03 INFO common.c: G070/G071/G081: 36 KiB SRAM, 128 KiB flash in at least 2 KiB pages.
2023-06-02T14:35:03 INFO common.c: G070/G071/G081: 36 KiB SRAM, 128 KiB flash in at least 2 KiB pages.
2023-06-02T14:35:03 INFO common.c: Attempting to write 28884 (0x70d4) bytes to stm32 address: 134217728 (0x8000000)
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08000000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08000800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08001000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08001800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08002000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08002800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08003000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08003800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08004000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08004800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08005000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08005800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08006000 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08006800 erased
2023-06-02T14:35:03 INFO common.c: Flash page at addr: 0x08007000 erased
2023-06-02T14:35:03 INFO common.c: Finished erasing 15 pages of 2048 (0x800) bytes
2023-06-02T14:35:03 INFO common.c: Starting Flash write for WB/G0/G4
2023-06-02T14:35:07 INFO common.c: Starting verification of write complete
2023-06-02T14:35:07 INFO common.c: Flash written and verified! jolly good!
````

# TEST

Use tio utility to connect to nucleo's serial port

```
tio /dev/ttyACM0 
[14:37:03.478] tio v2.5
[14:37:03.478] Press ctrl-t q to quit
[14:37:03.479] Connected
G071> help
avail cmds: uptime help 
G071> uptime
up 120 secs
```
