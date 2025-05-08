# 🔐 TB-03F-OpenHaystack-Firmware
**Openhaystack firmware for the Telink TLSR8253 MCU / TB-03F**

This project is a custom firmware implementation that brings OpenHaystack support to the TLSR825x chip.

---

## ✨ Features

- ✅ **Apple FindMy Network Support** 
  Seamlessly integrates with Apple's FindMy ecosystem for secure location sharing.

- 🔄 **Rotating Public Keys** *(NEW)* 
  Implements rotating identity keys for enhanced user privacy and FindMy compliance.

- 🔋 **Battery Status Reporting** *(NEW)* 
  Reports battery voltage over UART, complete with status flags (Low, Medium, Full, Critical) and debug logging.

---

## Installation
These instructions require linux. Although they almost can't be called instructions, since it's just cloning three repositories and unpacking the toolchain tarball to `/opt`:
```bash
git clone https://github.com/
wget http://shyboy.oss-cn-shenzhen.aliyuncs.com/readonly/tc32_gcc_v2.0.tar.bz2
sudo tar -xvjf　tc32_gcc_v2.0.tar.bz2　-C /opt/
export PATH=$PATH:/opt/tc32/bin
```
For convenience you could add the PATH export to whatever gets called on starting a new shell, so it's always available.

## Further preparation
1. Generate a `.keys` file with `FindMy/generate_keys.py` with three keypairs.
2. Add the key to the firmware using your brain and patience
3. Flash the new `FindMy.bin` file using `tb03f-pyflasher.py` over an UART, for example an USB UART dongle:
```
python tb03f-pyflasher.py --port /dev/ttyUSB0 FindMy.bin
```

## Retrieving the location of the module
This is explained in more detail in the `FindMy` repository, but boils down to running the `request_reports.py` script in the same
directory as the `.keys` file generated here.
