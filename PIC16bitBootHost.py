'''
Boot Host for Microchips 16-bootloader with UART support

Copyright (C) 2023, bassbreaker

This software is free for pretty much any one to use. That is unless you're from Microchip. In that case
at least reach out to me.
'''

import enum
import intelhex
import serial
import struct
import sys

HEX_FILENAME = 'hex_to_write.hex'
DEV_PORT = '/dev/ttyUSB0'

CMD_VER = b'\x00'
CMD_READ = b'\x01'
CMD_WRITE = b'\x02'
CMD_ERASE = b'\x03'
CMD_CHKSM = b'\x08'
CMD_RESET = b'\x09'
CMD_VERIFY = b'\x0A'
CMD_MEMORY = b'\x0B'
UNLOCK_KEY = b'\x55\x00\xAA\x00'
EMPTY_BYTE = b'\x00\x00'


class MCUResponse(enum.Enum):
    SUCCESS = 0x01
    INVALID_COMPARE = 0xFD
    INVALID_ADDRESS = 0XFE
    UNSUPPORTED_CMD = 0XFF


def print_progress_bar(iteration, total, prefix='', suffix='', decimals=1, length=100, fill='█'):
    percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print('\r%s |%s| %s%% %s \r' % (prefix, bar, percent, suffix), end=""),
    if iteration == total:
        print()


class BootDevice:
    def __init__(self):
        self.port = serial.Serial()
        self.bootloader_version = 0
        self.max_packet_size = 0
        self.device_id = 0
        self.erase_page_size = 0
        self.min_write_size = 0
        self.start_address = 0
        self.end_address = 0

    def open_port(self, device='/dev/ttyUSB0', baudrate=115200):
        self.port = serial.Serial(device, baudrate, timeout=1)

    def close_port(self):
        self.port.close()

    def get_version(self) -> bytes:
        self.port.write(CMD_VER + 5 * EMPTY_BYTE)
        resp = self.port.read(37)
        _, self.bootloader_version, self.max_packet_size, _, self.device_id, _, self.erase_page_size, \
            self.min_write_size, _ = struct.unpack('=11pHH2pH2pHH12p', resp)
        return resp

    def get_memory_range(self) -> bytes:
        self.port.write(CMD_MEMORY + b'\x08\x00' + 4 * EMPTY_BYTE)
        head = self.port.read(12)
        resp = self.port.read(8)
        self.start_address, self.end_address = struct.unpack('=LL', resp)
        return resp

    def erase_mcu(self, start_addr, num_pages) -> MCUResponse:
        print(f"Erasing data from 0x{start_addr:04x} to 0x{start_addr + num_pages * self.erase_page_size:04x}")
        self.port.write(struct.pack('=cH4sI', CMD_ERASE, num_pages, UNLOCK_KEY, start_addr))
        resp = self.port.read(12)
        return MCUResponse(resp[11])

    # Erase full space defined by start and end addresses from mcu
    def erase_full_mcu(self) -> MCUResponse:
        if self.erase_page_size == 0:
            sys.stderr.write("Please get version of MCU Bootloader\n")
            return
        pages_to_erase = (self.end_address - self.start_address) // self.erase_page_size
        return self.erase_mcu(self.start_address, pages_to_erase)

    def reset_mcu(self) -> MCUResponse:
        self.port.write(struct.pack("=cHII", CMD_RESET, 0, 0, 0))
        print("Resetting MCU")
        resp = self.port.read(12)
        return MCUResponse(resp[11])

    def write_hex_file(self, hex_filename):
        if self.erase_page_size == 0 or self.start_address == 0:
            sys.stderr.write("Please initialize MCU\n")
            return
        hex_data = intelhex.IntelHex(hex_filename)
        hex_last_addr = 0
        # Make sure that start addr in hex file matches bootloader
        for idx, segs in enumerate(hex_data.segments()):
            if segs[0] == self.start_address * 2:
                hex_last_addr = segs[1] // 2
                break
        if hex_last_addr == 0:
            sys.stderr.write("ERROR: Hex file segment not aligned with MCU Bootloader\n")
            return
        head_size = 11
        max_data_packet = ((self.max_packet_size - head_size) // self.min_write_size) * self.min_write_size
        # Pad the hex file with 0xFF to prevent error when trying to access memory that isn't there
        if (hex_last_addr//max_data_packet) * max_data_packet != hex_last_addr:
            good_last_addr = (hex_last_addr//max_data_packet + 1) * max_data_packet
            for addr in range(hex_last_addr*2, good_last_addr*2):
                hex_data.puts(addr, b'\xFF')
            hex_last_addr = good_last_addr
        print("Writing Program")
        total_blocks = (hex_last_addr-self.start_address)//(max_data_packet//2)
        for idx, addr in enumerate(range(self.start_address, hex_last_addr, max_data_packet//2)):
            print_progress_bar(idx+1, total_blocks, prefix="Programing", length=50)
            data = hex_data.tobinstr(start=addr*2, size=max_data_packet)
            if self.write_to_mcu(addr, data) != MCUResponse.SUCCESS:
                print(f"Error writing to {addr}")
        print("Write Complete")

    def read_from_mcu(self, addr, num_bytes):
        head_size = 11
        if num_bytes - head_size > self.max_packet_size:
            sys.stderr.write("num_bytes too large\n")
            return
        if num_bytes % 4 != 0:
            sys.stderr.write("num_bytes must be mod 4\n")
            return
        self.port.write(struct.pack("=cH4sI", CMD_READ, num_bytes, UNLOCK_KEY, addr))
        resp = self.port.read(12)
        if MCUResponse(resp[11]) != MCUResponse.SUCCESS:
            return MCUResponse(resp[11])
        return self.port.read(num_bytes)

    def write_to_mcu(self, addr, bytes_to_write):
        head_size = 11
        if len(bytes_to_write) + head_size > self.max_packet_size:
            sys.stderr.write("bytes_to_write too large\n")
            return
        if len(bytes_to_write) % self.min_write_size != 0:
            sys.stderr.write(f"bytes_to_write must be mod {self.min_write_size}\n")
            return
        head = struct.pack('=cH4sI', CMD_WRITE, len(bytes_to_write), UNLOCK_KEY, addr)
        self.port.write(head + bytes_to_write)
        resp = self.port.read(12)
        return MCUResponse(resp[11])


if __name__ == '__main__':
    bd = BootDevice()
    bd.open_port(device=DEV_PORT)
    ver_bytes = bd.get_version()
    mem_range = bd.get_memory_range()
    bd.erase_full_mcu()
    bd.write_hex_file(HEX_FILENAME)
    bd.reset_mcu()
    bd.close_port()
