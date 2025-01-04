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


class LN882FirmwareUploader(object):
    
    def __init__(self):
        print('Init')
        
    # open the comport
    def open(self, port, baudrate=115200):
        
        print ('Try to open port %s. Press ctrl+c for break' % port)
        while 1:
            try:
                self.ser = serial.Serial(port, baudrate, timeout=1, dsrdtr=True, rtscts=True)
                print ('Connect to Port %s' % port)
                break
            except:
                time.sleep(0.1) 
                continue
        print("Port open")

    def getc(self, size, timeout=1):
        return self.ser.read(size) or None
    
    def putc(self, data, timeout=1):
        return self.ser.write(data)
    
    """ upload flash tool in RAM """
    def uploadRamLoader(self,filename):

        print ('Sync with LN882... wait 10 seconds')
        self.ser.flushInput()
        time.sleep(5)
        
        msg=''
        while (msg != b'Mar 14 2021/00:23:32\r\n'):
            time.sleep(2)
            self.flushCom()
            print ('send version... wait for:  Mar 14 2021/00:23:32')
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)
               
        print ('Connect to bootloader...')
        self.ser.write(bytes(b'download [rambin] [0x20000000] [37872]\r\n'))
        print("Send file")
        
        modem = YModem(self.getc, self.putc)
    
        modem.send_file(filename,False,3)
    
        print("Start program. Wait 10 seconds")
        time.sleep(5)
        
        msg = ''
        while (msg != b'RAMCODE\r\n'):
            time.sleep(5)
            self.ser.flushInput()
            print ('send version... wait for:  RAMCODE')
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)
            msg = self.ser.readline()
            print(msg)
                 
        self.ser.write(bytes(b'flash_uid\r\n'))
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip()) 
    
    def changeBaudrate(self,baudrate):
          # change baudrate
        print('Change baudrate ' + str(baudrate))  
        msg =  'baudrate ' + str(baudrate) +'\r\n' 
        self.ser.write(msg.encode(encoding="utf-8"))
        
        #read echo
        msg = self.ser.read(15)
        print(msg)
        self.ser.baudrate = baudrate
    
        print('Wait 5 seconds for sync')
        time.sleep(5)
        self.flushCom()

        # ping and sync
        msg=''
        while (msg != b'RAMCODE\r\n'):
            print ('send version... wait for:  RAMCODE')
            time.sleep(1)
            self.flushCom()
            self.ser.write(bytes(b'version\r\n'))
            msg = self.ser.readline()
            print(msg)
            msg = self.ser.readline()
            print(msg)
           
        print('Done')
        
    
    def flashProgram(self,port,filename):      
        # change baudrate
        self.changeBaudrate(921600)
        modem = YModem(self.getc, self.putc)
        
        # set flash start adress
        self.ser.write(bytes(b'startaddr 0x0\r\n'))
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())
        
        msg = self.ser.readline().decode("utf-8")
        print(msg.strip())
        
        self.ser.write(bytes(b'upgrade\r\n'))
        # read echo
        msg = self.ser.read(7)
        
        # upload file. 16k packet size
        modem.send_file(filename,True,3)
        
        self.ser.write(bytes(b'filecount\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())
        
        self.changeBaudrate(115200)        
    
    def flashInfo(self):
        self.ser.write(bytes(b'flash_info\r\n'))
        msg = self.ser.readline()
        print(msg.strip())
        msg = self.ser.readline()
        print(msg.strip())
        
    def getMacInOTP(self):
      self.ser.write(bytes(b'get_mac_in_flash_otp\r\n'))
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
      
    def readFlashOTP(self):
      self.ser.write(bytes(b'flash_ot p_read 0 256\r\n'))
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
      
    def getOTPLock(self):
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
 
    def readGpio(self, pin ):
      msg = 'gpio_read ' + pin +  '\r\n'
      self.ser.write(msg.encode(encoding="utf-8"))
      #Echo
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
    
    def writeGpio(self, pin ,val):
      msg = 'gpio_write ' + pin + ' ' + val +  '\r\n'
      self.ser.write(msg.encode(encoding="utf-8"))
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
    
    ''' The size of the read must be less than 0x100. '''
    def readFlash(self):
      self.ser.write(bytes(b'flash_read 0x0 0xFF\r\n'))
      #Echo
      msg = self.ser.readline()
      print(msg.strip())
      # Flash content
      msg = self.ser.readline()
      print(msg.strip())
      
    ''' Dump flash as intel hex'''
    def dumpFlash(self):
      self.ser.write(bytes(b'fdump 0x0 0x2000\r\n'))
      #Echo
      msg = self.ser.readline()
      print(msg.strip())
      # Flash content
      while(1):
        msg = self.ser.readline().strip()
        if(msg == b'pppp'):
            break
        print(msg)
        
            
    def getGpioAll(self):
      self.ser.write(bytes(b'gpio_read_al\r\n'))
      msg = self.ser.readline()
      print(msg.strip())
      msg = self.ser.readline()
      print(msg.strip())
               
    def close(self):
        self.ser.close()
                
    def flushCom(self):
        self.ser.flushInput()
        self.ser.flushOutput()
    

def main():

    parser = argparse.ArgumentParser(description='Firmware uploader for LN882.', prog='loader')
    parser.add_argument('--port', '-p', help='Serial port device', default='/dev/ttyUSB0')
    parser.add_argument('--flashfile', '-f', help='Upload firmware file', default='OpenLN882H_1.17.769.bin')
    args = parser.parse_args()
         
    if not os.path.isfile(args.flashfile):
        raise Exception('Can not open file: ' + args.flashfile)
        
    h = LN882FirmwareUploader()
       
    h.open(args.port)
    
    h.uploadRamLoader('LN882H_RAM_BIN.bin')
    
    #h.flashInfo()
    #h.readFlash()
    #h.dumpFlash()
    #h.getMacInOTP()
    #h.writeGpio('A8','1')    
    #h.readGpio('A1')
    #h.readGpio('A6')
    #h.readGpio('A7')
    
    #h.getGpioAll()
    #h.getTest()
    
    h.flashProgram(args.port, args.flashfile)
         
    h.close()

    
if __name__ == '__main__':
    try:
        main()
       
    except Exception:
        traceback.print_exc()
