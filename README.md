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

## Embedded ROMs
When the firmware image is built, ROM files that are put in the src/roms directory before CMake reconfiguring, get packaged with the firmware image. Currently each ROM file needs dedicated code in the firmware to be useful so only the most critical ROM files should be integrated. This is for now limited to the LOCI ROM and Mike Brown's great Oric diagnostic ROM. The release builds include these ROMs, but they are separate dat objects amd projects, not part of the LOCI firmware project and no rights implied by the integrations.

### LOCI ROM 
https://github.com/sodiumlb/loci-rom

The official ROM image for configuring LOCI. Boots by normal short press of the LOCI action button.

### Mike Brown's Oric Diagnostic ROM
https://oric.signal11.org.uk/html/diagrom.htm

Mike Brown has agreed to this project including the diagnostic ROM with release builds so more Orics can be saved. In many cases, this can limit the need for burning  EPROM chips or creating special ROM expansion boards. Long pressing the action button on LOCIs with diagrom enabled firmware will boot the diagrom instead of the LOCI ROM.

Having the diagrom available with LOCI does not remove the need for reading the diagrom documentation and using proper test harnesses as described in the documentation. It is not a full substitution for the diagnostic PCB. While having the diagrom with LOCI can make it easier to start trouble shooting an Oric computer, remember it also introduces itself as a potential fault source. Following the diagrom documentation step-by-step is key to understanding where a troubled Oric is failing. Read more on the [Oric/Atmos diagnostic ROM and PCB](https://oric.signal11.org.uk/html/diagrom.htm) homepage.

Thanks to Mike Brown for this contribution!
