#!/usr/bin/env python3
#
# Copyright (C) 2025  Stefan Mandl
#
# Firmware flasher for LN882H
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 2 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc., 51 Franklin
# Street, Fifth Floor, Boston, MA 02110-1301 USA.


# https://github.com/openshwprojects/OpenBK7231T_App


import argparse
import os
import sys
import time
import traceback
try:
    import serial
except ImportError:
    print('Please install pySerial ( pip3 install pyserial)')
    sys.exit()

from YModem import YModem


class LN882FirmwareUploader:
    ''' Flashing and dumping tool for LN882H'''

    def __init__(self):
        print('Init')
        self.ser = None

    def open(self, port, baudrate=115200):
        '''open the comport'''

        print('Try to open port %s. Press ctrl+c for break' % port)
        while 1:
            try:
                self.ser = serial.Serial(
                    port, baudrate, timeout=1, dsrdtr=True, rtscts=True)
                print('Connect to Port %s' % port)
                break
            except:
                time.sleep(0.1)
                continue
        print("Port open")

    def getc(self, size, timeout=1):
        ''' getc for ymodem'''
        return self.ser.read(size) or None

    def putc(self, data, timeout=1):
        ''' putc for ymodem'''
        return self.ser.write(data)

    def upload_ram_loader(self, filename):
        """ upload flash tool in RAM """

        print('Sync with LN882H... wait 5 seconds')
        self.ser.flushInput()
        time.sleep(5)

        msg = ''
        while msg != b'Mar 14 2021/00:23:32\r\n':
            time.sleep(2)
            self.flush_com()
            print('send version... wait for:  Mar 14 2021/00:23:32')
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)

        print('Connect to bootloader...')
        self.ser.write(bytes(b'download [rambin] [0x20000000] [37872]\r\n'))
        print("Send file")

        modem = YModem(self.getc, self.putc)

        # send file. using ymodem
        # https://en.wikipedia.org/wiki/YMODEM
        modem.send_file(filename, False, 3)

        # wait for ramloader is running
        print("Start program. Wait 5 seconds")
        time.sleep(5)

        # check
        msg = ''
        while msg != b'RAMCODE\r\n':
            time.sleep(5)
            self.ser.flushInput()
            print('send version... wait for:  RAMCODE')
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)
            msg = self.ser.readline()
            print(msg)

        # get flash uid
        self.ser.write(bytes(b'flash_uid\r\n'))
        msg = self.ser.readline().decode("utf-8")
        # print(msg.strip())
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())

    def change_baudrate(self, baudrate):
        ''' Change baudrate '''
        # send change baudrate command
        print('Change baudrate ' + str(baudrate))
        msg = 'baudrate ' + str(baudrate) + '\r\n'
        self.ser.write(msg.encode(encoding="utf-8"))

        # read echo
        msg = self.ser.read(15)
        print(msg)

        # change my baudrate
        self.ser.baudrate = baudrate

        print('Wait 5 seconds for change')
        time.sleep(5)
        self.flush_com()

        # ping and sync
        msg = ''
        while msg != b'RAMCODE\r\n':
            print('send version... wait for:  RAMCODE')
            time.sleep(1)
            self.flush_com()
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)
            msg = self.ser.readline()
            print(msg)

        print('Baudrate change done')

    def flash_program(self, filename):
        ''' Flash new firmware version '''
        # change baudrate
        self.change_baudrate(921600)
        modem = YModem(self.getc, self.putc)

        # set flash start adress
        self.ser.write(bytes(b'startaddr 0x0\r\n'))
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())

        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())

        # send upgrade command
        self.ser.write(bytes(b'upgrade\r\n'))
        # read echo
        msg = self.ser.read(7)

        # upload file using ymodem. 16k packet size
        modem.send_file(filename, True, 3)

        # get upload file count
        self.ser.write(bytes(b'filecount\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

        # change baudrate back
        self.change_baudrate(115200)

    def flash_erase_all(self):
        '''
        Erase flash 

        '''
        self.ser.write(bytes(b'ferase_all\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def flash_info(self):
        '''
        Read flash info
        
        LN882HKI	QFN-32, 5mm pitch	2MB Flash
        
        LN882HKG	QFN-32, 5mm pitch	1MB Flash

        Sample id:0xEB6015,flash size:2M Byte
        
        '''
        self.ser.write(bytes(b'flash_info\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def get_mac_in_otp(self):
        ''' get mac otp '''
        self.ser.write(bytes(b'get_mac_in_flash_otp\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def get_mac_local(self):
        ''' get mac local '''
        self.ser.write(bytes(b'get_m_local_mac\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def read_flash_otp(self):
        ''' read flash otp '''
        self.ser.write(bytes(b'flash_ot p_read 0 256\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def get_otp_lock(self):
        ''' read otp lock '''
        self.ser.write(bytes(b'flash_ot p_get_lock_state\r\n'))
        # echo
        msg = self.ser.readline()
        print(msg.strip())
        # value
        msg = self.ser.readline()
        print(msg.strip())

        # empty
        msg = self.ser.readline()
        print(msg.strip())

        # pppp
        msg = self.ser.readline()
        print(msg.strip())

    def read_gpio(self, pin):
        ''' read gpio pin '''
        msg = 'gpio_read ' + pin + '\r\n'
        self.ser.write(msg.encode(encoding="utf-8"))
        # Echo
        msg = self.ser.readline()
        print(msg.strip())
        # value
        msg = self.ser.readline()
        print(msg.strip())
        #
        msg = self.ser.readline()
        print(msg.strip())
        # pppp
        msg = self.ser.readline()
        print(msg.strip())

    def write_gpio(self, pin, val):
        ''' write gpio pin '''
        msg = 'gpio_write ' + pin + ' ' + val + '\r\n'
        self.ser.write(msg.encode(encoding="utf-8"))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def read_flash(self, flash_addr, is_otp=False):
        '''
        Read flash address
        The size of the read must be less than 0x100.
        Last 2 byte CRC16 checksum

        Parameters
        ----------

        flash_addr    Read from this start address

        is_otp        False      read from flash
                      True       read from OTP 

        '''
        flash_data = bytearray()
        if is_otp:
            cmd = 'flash_otp_read ' + hex(flash_addr) + ' 0x100\r\n'
        else:
            cmd = 'flash_read ' + hex(flash_addr) + ' 0x100\r\n'

        self.ser.write(cmd.encode(encoding="utf-8"))
        # Echo
        msg = self.ser.readline()
        # Flash content
        msg = self.ser.readline()
        # print(msg.strip())

        line = msg.strip().decode('utf-8')
        line = line.strip().replace(' ', '')  # Remove spaces
        line_len = len(line)  # check line length

        if line_len != 256 * 2 + 4:
            print()
            print('Lost bytes. Got only: %s' % line_len)
            return False, flash_data

        checksum = line[-4:]
        # print(checksum)

        linedata = line[:-4]  # Remove checksum
        flash_data = bytearray.fromhex(linedata)
        bincrc = YModem.calc_crc(YModem, flash_data)

        # print(hex(bincrc))
        if int(checksum, 16) != bincrc:
            print()
            print("Checksum fail")
            return False, flash_data

        return True, flash_data

    def read_flash_to_file(self, filename, flash_size, is_otp=False):
        ''' 
        Dump flash or OTP to file

        filename     
        flash_size        for OTP max 1k
                          for flash 2 MB

        Downloading is very slow. We can only download 0xFF byte in one cycle
        '''
        print("Reading flash to file %s" % filename)
        flash_addr = 0
        with open(filename, "wb") as flash_file:
            while flash_addr < flash_size:
                ok, flash_data = self.read_flash(flash_addr, is_otp)
                if ok is True:
                    flash_file.write(flash_data)
                print('.', end='', flush=True)
                flash_addr += 0x100
        print()
        print('done')

    def dump_flash(self):
        ''' Dump flash as hex'''
        self.ser.write(bytes(b'fdump 0x0 0x2000\r\n'))
        # Echo
        msg = self.ser.readline()
        print(msg.strip())
        # Flash content
        while True:
            msg = self.ser.readline().strip()
            if msg == b'pppp':
                break
            print(msg)

    def get_gpio_all(self):
        ''' Real all gpio pins'''
        self.ser.write(bytes(b'gpio_read_al\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())

    def close(self):
        ''' close serial '''
        self.ser.close()

    def flush_com(self):
        ''' flush all'''
        self.ser.flushInput()
        self.ser.flushOutput()


def main():
    ''' LN882H tool '''

    parser = argparse.ArgumentParser(
        description='Firmware uploader for LN882H.', prog='loader')
    parser.add_argument(
        '--port', '-p', help='Serial port device', default='/dev/ttyUSB0')
    parser.add_argument(
        '--flashfile', '-f', help='Upload firmware file', default='OpenLN882H_1.18.7.bin')
    args = parser.parse_args()

    if not os.path.isfile(args.flashfile):
        raise Exception('Can not open file: ' + args.flashfile)

    h = LN882FirmwareUploader()

    h.open(args.port)

    # load RAMCode so that we can access flash
    h.upload_ram_loader('LN882H_RAM_BIN.bin')

    # h.flash_info()
    # h.read_flash(0)
    # h.get_mac_local()
    # h.flash_erase_all()
    # h.change_baudrate(921600)
    # h.change_baudrate(2000000)
    # h.read_flash_to_file("dump.bin", 0x200000)
    # h.read_flash_to_file("dump_otp.bin", 0x400, True)
    # h.change_baudrate(921600)
    # h.dump_flash()
    # h.get_mac_in_otp()
    # h.write_gpio('A8', '1')
    # h.read_gpio('A1')
    # h.read_gpio('A6')
    # h.read_gpio('A7')
    # h.get_gpio_all()

    h.flash_program(args.flashfile)

    h.close()


if __name__ == '__main__':
    try:
        main()

    except Exception:
        traceback.print_exc()
