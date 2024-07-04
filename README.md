# Oric LOCI Firmware

This is the firmware source for the Oric LOCI, a modern storage emulation device for the Oric range of 8-bit computers like Oric-1 and Oric Atmos. It is based on Rumbledethumps' Picocomputer 6502 (RP6502) https://picocomputer.github.io/ but uses the Oric computers for the 6502 and video side of the project. Further the RIA has been adopted to interface with the Oric expansion bus and emulate floppy drives and tape loading images, making it a MIA (Media Interface Adapter)

## Dev Setup

This is only for building the LOCI firmware.

Install the C/C++ toolchain for the Raspberry Pi Pico. For more information, read [Getting started with the Raspberry Pi Pico](https://rptl.io/pico-get-started).
```
sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential gdb-multiarch
```

All dependencies are submodules. The following will download the correct version of all SDKs. It will take an extremely long time to recurse the project, so do this instead:
```
git submodule update --init
cd src/pico-sdk
git submodule update --init
cd ../..
```

To build from the command line in a separate build directory:
```
mkdir build
cd build
cmake ../
make
```
