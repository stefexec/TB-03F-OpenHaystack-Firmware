#!/usr/bin/env python3
"""
TLSR8253 Flasher - PySerial Version
Based on ATC1441's work and the provided web flasher

Dependencies:
- pyserial: pip install pyserial
"""

import argparse
import os
import struct
import sys
import time
from typing import List, Optional, Union

import serial
import serial.tools.list_ports


class TLSR8253Flasher:
    """TLSR8253 Flash tool using PySerial"""

    def __init__(self, port: str, baudrate: int = 460800):
        """Initialize the flasher with the specified port and baudrate"""
        self.port = port
        self.baudrate = baudrate
        self.ser = None

    def open(self) -> bool:
        """Open the serial port"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
            # Default state: DTR and RTS off
            self.ser.setDTR(False)
            self.ser.setRTS(False)
            return True
        except Exception as e:
            print(f"Error opening serial port: {e}")
            return False

    def close(self) -> None:
        """Close the serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial port closed")

    def reset_mcu(self, duration_ms: int = 100) -> None:
        """Reset the MCU using DTR/RTS lines"""
        print(f"Resetting device via DTR/RTS ({duration_ms} ms)")
        if self.ser:
            self.ser.setDTR(True)
            self.ser.setRTS(True)
            time.sleep(duration_ms / 1000.0)
            self.ser.setDTR(False)
            self.ser.setRTS(False)

    def sws_wr_addr(self, addr: int, data: Union[bytes, bytearray, List[int]]) -> bytes:
        """Generate SWS packet to write data to address"""
        if isinstance(data, list):
            data = bytes(data)
        elif not isinstance(data, (bytes, bytearray)):
            data = bytes([data])

        d = bytearray(10)
        h = bytearray([0x5A, (addr >> 16) & 0xFF, (addr >> 8) & 0xFF, addr & 0xFF, 0x00])
        pkt = bytearray((len(data) + 6) * 10)
        
        # Start bit byte cmd swire = 1
        d[0] = 0x80
        # Stop bit swire = 0
        d[9] = 0xFE
        
        for n, el in enumerate(h):
            m = 0x80  # mask bit
            idx = 1
            while m != 0:
                if (el & m) != 0:
                    d[idx] = 0x80
                else:
                    d[idx] = 0xFE
                idx += 1
                m >>= 1
            pkt[n * 10:(n * 10) + 10] = d
            # Start bit next byte swire = 0
            d[0] = 0xFE
        
        for n, el in enumerate(data):
            m = 0x80  # mask bit
            idx = 1
            while m != 0:
                if (el & m) != 0:
                    d[idx] = 0x80
                else:
                    d[idx] = 0xFE
                idx += 1
                m >>= 1
            pkt[(n + 5) * 10:((n + 5) * 10) + 10] = d
        
        # swire stop cmd = 0xff
        d = bytearray([0x80] * 9 + [d[9]])
        pkt[(len(data) + 5) * 10:((len(data) + 5) * 10) + 10] = d
        
        return bytes(pkt)

    def write_raw(self, data: bytes) -> None:
        """Write raw data to the serial port"""
        if self.ser:
            self.ser.write(data)

    async_def = lambda self, f: f  # Dummy async for porting from web version

    def flash_byte_cmd(self, cmd: int) -> None:
        """Send a single byte command to the flash"""
        # CNS low
        self.write_raw(self.sws_wr_addr(0x0D, [0x00]))
        # Flash cmd + CNS high
        self.write_raw(self.sws_wr_addr(0x0C, [cmd & 0xFF, 0x01]))

    def flash_write_enable(self) -> None:
        """Enable flash writing"""
        self.flash_byte_cmd(0x06)

    def flash_wake_up(self) -> None:
        """Wake up the flash chip"""
        self.flash_byte_cmd(0xAB)

    def flash_unlock(self) -> None:
        """Unlock the flash for writing"""
        self.flash_write_enable()
        
        # CNS low
        self.write_raw(self.sws_wr_addr(0x0D, [0x00]))
        # Flash cmd
        self.write_raw(self.sws_wr_addr(0x0C, [0x01]))
        # Unlock all + CNS high
        self.write_raw(self.sws_wr_addr(0x0C, [0x00, 0x01]))
        
        self.flash_write_enable()
        
        # CNS low
        self.write_raw(self.sws_wr_addr(0x0D, [0x00]))
        # Flash cmd
        self.write_raw(self.sws_wr_addr(0x0C, [0x01]))
        # Unlock all
        self.write_raw(self.sws_wr_addr(0x0C, [0x00]))
        # Unlock all + CNS high
        self.write_raw(self.sws_wr_addr(0x0C, [0x00, 0x01]))

    def write_fifo(self, addr: int, data: bytes) -> None:
        """Write data to FIFO"""
        # [0xB3]=0x80 ext.SWS into fifo mode
        self.write_raw(self.sws_wr_addr(0x00B3, [0x80]))
        # Send all data to one register (no increment address - fifo mode)
        self.write_raw(self.sws_wr_addr(addr, data))
        # [0xB3]=0x00 ext.SWS into normal(ram) mode
        self.write_raw(self.sws_wr_addr(0x00B3, [0x00]))

    def sector_erase(self, addr: int) -> None:
        """Erase a sector at the specified address"""
        self.flash_write_enable()
        
        # CNS low
        self.write_raw(self.sws_wr_addr(0x0D, [0x00]))
        # Flash cmd erase sector
        self.write_raw(self.sws_wr_addr(0x0C, [0x20]))
        # Faddr hi
        self.write_raw(self.sws_wr_addr(0x0C, [(addr >> 16) & 0xFF]))
        # Faddr mi
        self.write_raw(self.sws_wr_addr(0x0C, [(addr >> 8) & 0xFF]))
        # Faddr lo + CNS high
        self.write_raw(self.sws_wr_addr(0x0C, [addr & 0xFF, 0x01]))
        
        # Wait for erase to complete
        time.sleep(0.3)

    def write_flash_blk(self, addr: int, data: bytes) -> None:
        """Write a block of data to flash"""
        self.flash_write_enable()
        
        # CNS low
        self.write_raw(self.sws_wr_addr(0x0D, [0x00]))
        
        blk = bytearray(4 + len(data))
        blk[0] = 0x02
        blk[1] = (addr >> 16) & 0xFF
        blk[2] = (addr >> 8) & 0xFF
        blk[3] = addr & 0xFF
        blk[4:] = data
        
        # Send all data to SPI data register
        self.write_fifo(0x0C, blk)
        
        # CNS high
        self.write_raw(self.sws_wr_addr(0x0D, [0x01]))
        
        # Wait for write to complete
        time.sleep(0.01)

    def soft_reset_mcu(self) -> None:
        """Perform a soft reset of the MCU"""
        self.write_raw(self.sws_wr_addr(0x06F, [0x20]))

    def activate(self, duration_ms: int) -> None:
        """Activate the device for programming"""
        # CPU stop command
        cpu_stop_cmd = self.sws_wr_addr(0x0602, [0x05])
        
        print(f"Reset DTR/RTS (100 ms)")
        self.reset_mcu(100)
        
        print("Soft Reset MCU")
        self.soft_reset_mcu()
        
        print(f"Activate ({duration_ms/1000.0} sec)...")
        start_time = time.time()
        while (time.time() - start_time) < (duration_ms / 1000.0):
            self.write_raw(cpu_stop_cmd)  # CPU stop
        
        # Set SWS Speed
        self.write_raw(self.sws_wr_addr(0x00B2, [55]))
        self.write_raw(cpu_stop_cmd)
        self.flash_wake_up()

    def is_telink_firmware(self, data: bytes) -> bool:
        """Check if the firmware is a valid Telink firmware"""
        if len(data) < 16:
            return False
        
        # Check for TLNK signature at offset 8
        signature = struct.unpack_from('<I', data, 8)[0]
        return signature == 0x544C4E4B  # "TLNK" in little endian

    def flash_firmware(self, firmware_data: bytes, activation_time_ms: int = 1000) -> bool:
        """Flash the provided firmware to the device"""
        if not self.is_telink_firmware(firmware_data):
            print("Error: Selected file is not a Telink firmware .bin")
            return False

        start_time = time.time()
        
        try:
            self.activate(activation_time_ms)
            self.flash_write_enable()
            self.flash_unlock()
            time.sleep(1.5)
            
            print(f"Writing {len(firmware_data)} bytes to Flash...")
            
            # Flash parameters
            addr = 0
            length = len(firmware_data)
            block_size = 256  # max spi-flash fifo = 256
            
            while length > 0:
                # Erase sector (4KB) when needed
                if (addr & 0x0FFF) == 0:
                    progress = int(addr / len(firmware_data) * 100)
                    print(f"{progress}% Flash Sector Erase at 0x{addr:06X}")
                    self.sector_erase(addr)
                
                # Adjust block size for remaining data
                if length < block_size:
                    block_size = length
                
                # Write block
                progress = int(addr / len(firmware_data) * 100)
                print(f"{progress}% Flash Write {block_size} bytes at 0x{addr:06X}")
                self.write_flash_blk(addr, firmware_data[addr:addr + block_size])
                
                # Update counters
                addr += block_size
                length -= block_size
            
            # Perform soft reset after flashing
            print("Flashing complete!")
            print(f"Done ({time.time() - start_time:.2f} sec)")
            print("Soft Reset MCU")
            self.soft_reset_mcu()
            return True
            
        except Exception as e:
            print(f"Error during flashing: {e}")
            return False


def list_ports() -> None:
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("No serial ports found.")
        return
    
    print("\nAvailable serial ports:")
    for port in ports:
        print(f"  {port.device} - {port.description}")
    print()


def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description="TLSR8253 Flasher - PySerial Version")
    parser.add_argument("-p", "--port", help="Serial port")
    parser.add_argument("-b", "--baudrate", type=int, default=460800, 
                        help="Baudrate (default: 460800)")
    parser.add_argument("-a", "--activation-time", type=int, default=1000,
                        help="Activation time in ms (default: 1000)")
    parser.add_argument("-l", "--list-ports", action="store_true", 
                        help="List available serial ports")
    parser.add_argument("firmware", nargs="?", help="Firmware file to flash")
    
    args = parser.parse_args()
    
    if args.list_ports:
        list_ports()
        return 0
    
    if not args.port:
        print("Error: Serial port must be specified")
        parser.print_help()
        return 1
    
    if not args.firmware:
        print("Error: Firmware file must be specified")
        parser.print_help()
        return 1
    
    if not os.path.isfile(args.firmware):
        print(f"Error: Firmware file '{args.firmware}' not found")
        return 1
    
    try:
        with open(args.firmware, "rb") as f:
            firmware_data = f.read()
    except Exception as e:
        print(f"Error reading firmware file: {e}")
        return 1
    
    flasher = TLSR8253Flasher(args.port, args.baudrate)
    
    if not flasher.open():
        return 1
    
    try:
        print(f"Flashing file: {args.firmware} ({len(firmware_data)} bytes)")
        
        if not flasher.is_telink_firmware(firmware_data):
            print("Warning: Selected file does not appear to be a Telink firmware .bin")
            response = input("Do you want to continue anyway? (y/N): ")
            if response.lower() != 'y':
                return 1
        
        success = flasher.flash_firmware(firmware_data, args.activation_time)
        if success:
            print("Flashing completed successfully")
            return 0
        else:
            print("Flashing failed")
            return 1
    
    finally:
        flasher.close()


if __name__ == "__main__":
    sys.exit(main())
