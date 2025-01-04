
# LN882 fashing tool for linux

Here I will show you how you can easily flash the new LN882 modules via UART, just like you would flash ESP8266. Flashing those modules will allow you to free them from the cloud and connect them to Home Assistant. 


https://github.com/openshwprojects/OpenBK7231T_App

## Datasheet
https://www.elektroda.com/rtvforum/topic4027545.html

## Hardware setup

You need to connect:

- Module TX0 -> UART RX
- Module RX 0-> UART TX
- Module GND -> UART GND
- Module VCC (3V3)  -> UART 3V3 (make sure your USB UART supports and is in 3V3 mode if it is selectable)
- Module A9 -> GND
A9 must remain LOW during whole flashing process (do not disconnect it!). 

![image](doc/03.png)


## Fashing new firmware




```
loader.py

Try to open port /dev/ttyUSB0. Press ctrl+c for break
Connect to Port /dev/ttyUSB0
Port open
Snyc with LN882... wait 10 seconds
send version... wait for:  Mar 14 2021/00:23:32
b'Mar 14 2021/00:23:32\r\n'
Connect to bootloader...
Send file
2025-01-01 09:36:35,983 - DEBUG - <<< 0x43
2025-01-01 09:36:35,984 - DEBUG - Packet 0 >>>
2025-01-01 09:36:35,997 - DEBUG - <<< 0x6
2025-01-01 09:36:35,998 - DEBUG - <<< 0x43
2025-01-01 09:36:36,062 - DEBUG - Packet 1 >>>
2025-01-01 09:36:36,090 - DEBUG - <<< ACK
2025-01-01 09:36:36,155 - DEBUG - Packet 2 >>>
2025-01-01 09:36:36,183 - DEBUG - <<< ACK
2025-01-01 09:36:36,247 - DEBUG - Packet 3 >>>
2025-01-01 09:36:36,274 - DEBUG - <<< ACK
2025-01-01 09:36:36,339 - DEBUG - Packet 4 >>>
2025-01-01 09:36:36,366 - DEBUG - <<< ACK
2025-01-01 09:36:36,431 - DEBUG - Packet 5 >>>
2025-01-01 09:36:36,458 - DEBUG - <<< ACK
2025-01-01 09:36:36,523 - DEBUG - Packet 6 >>>
2025-01-01 09:36:36,550 - DEBUG - <<< ACK
2025-01-01 09:36:36,615 - DEBUG - Packet 7 >>>
2025-01-01 09:36:36,642 - DEBUG - <<< ACK
2025-01-01 09:36:36,707 - DEBUG - Packet 8 >>>
2025-01-01 09:36:36,734 - DEBUG - <<< ACK
2025-01-01 09:36:36,799 - DEBUG - Packet 9 >>>
2025-01-01 09:36:36,826 - DEBUG - <<< ACK
2025-01-01 09:36:36,891 - DEBUG - Packet 10 >>>
2025-01-01 09:36:36,918 - DEBUG - <<< ACK
2025-01-01 09:36:36,983 - DEBUG - Packet 11 >>>
2025-01-01 09:36:37,010 - DEBUG - <<< ACK
2025-01-01 09:36:37,075 - DEBUG - Packet 12 >>>
2025-01-01 09:36:37,102 - DEBUG - <<< ACK
2025-01-01 09:36:37,167 - DEBUG - Packet 13 >>>
2025-01-01 09:36:37,194 - DEBUG - <<< ACK
2025-01-01 09:36:37,258 - DEBUG - Packet 14 >>>
2025-01-01 09:36:37,286 - DEBUG - <<< ACK
2025-01-01 09:36:37,351 - DEBUG - Packet 15 >>>
2025-01-01 09:36:37,378 - DEBUG - <<< ACK
2025-01-01 09:36:37,442 - DEBUG - Packet 16 >>>
2025-01-01 09:36:37,470 - DEBUG - <<< ACK
2025-01-01 09:36:37,534 - DEBUG - Packet 17 >>>
2025-01-01 09:36:37,562 - DEBUG - <<< ACK
2025-01-01 09:36:37,627 - DEBUG - Packet 18 >>>
2025-01-01 09:36:37,655 - DEBUG - <<< ACK
2025-01-01 09:36:37,719 - DEBUG - Packet 19 >>>
2025-01-01 09:36:37,747 - DEBUG - <<< ACK
2025-01-01 09:36:37,811 - DEBUG - Packet 20 >>>
2025-01-01 09:36:37,839 - DEBUG - <<< ACK
2025-01-01 09:36:37,903 - DEBUG - Packet 21 >>>
2025-01-01 09:36:37,931 - DEBUG - <<< ACK
2025-01-01 09:36:37,995 - DEBUG - Packet 22 >>>
2025-01-01 09:36:38,023 - DEBUG - <<< ACK
2025-01-01 09:36:38,087 - DEBUG - Packet 23 >>>
2025-01-01 09:36:38,115 - DEBUG - <<< ACK
2025-01-01 09:36:38,179 - DEBUG - Packet 24 >>>
2025-01-01 09:36:38,207 - DEBUG - <<< ACK
2025-01-01 09:36:38,271 - DEBUG - Packet 25 >>>
2025-01-01 09:36:38,299 - DEBUG - <<< ACK
2025-01-01 09:36:38,363 - DEBUG - Packet 26 >>>
2025-01-01 09:36:38,391 - DEBUG - <<< ACK
2025-01-01 09:36:38,455 - DEBUG - Packet 27 >>>
2025-01-01 09:36:38,483 - DEBUG - <<< ACK
2025-01-01 09:36:38,548 - DEBUG - Packet 28 >>>
2025-01-01 09:36:38,576 - DEBUG - <<< ACK
2025-01-01 09:36:38,640 - DEBUG - Packet 29 >>>
2025-01-01 09:36:38,668 - DEBUG - <<< ACK
2025-01-01 09:36:38,732 - DEBUG - Packet 30 >>>
2025-01-01 09:36:38,760 - DEBUG - <<< ACK
2025-01-01 09:36:38,824 - DEBUG - Packet 31 >>>
2025-01-01 09:36:38,852 - DEBUG - <<< ACK
2025-01-01 09:36:38,916 - DEBUG - Packet 32 >>>
2025-01-01 09:36:38,944 - DEBUG - <<< ACK
2025-01-01 09:36:39,008 - DEBUG - Packet 33 >>>
2025-01-01 09:36:39,036 - DEBUG - <<< ACK
2025-01-01 09:36:39,100 - DEBUG - Packet 34 >>>
2025-01-01 09:36:39,128 - DEBUG - <<< ACK
2025-01-01 09:36:39,192 - DEBUG - Packet 35 >>>
2025-01-01 09:36:39,220 - DEBUG - <<< ACK
2025-01-01 09:36:39,284 - DEBUG - Packet 36 >>>
2025-01-01 09:36:39,312 - DEBUG - <<< ACK
2025-01-01 09:36:39,376 - DEBUG - Packet 37 >>>
2025-01-01 09:36:39,404 - DEBUG - <<< ACK
2025-01-01 09:36:39,404 - DEBUG - EOF
2025-01-01 09:36:39,405 - DEBUG - >>> EOT
2025-01-01 09:36:39,407 - DEBUG - <<< 0x15
2025-01-01 09:36:39,407 - DEBUG - >>> EOT
2025-01-01 09:36:39,409 - DEBUG - <<< 0x6
2025-01-01 09:36:39,409 - DEBUG - <<< 0x43
2025-01-01 09:36:39,410 - DEBUG - Packet End >>>
2025-01-01 09:36:39,423 - DEBUG - <<< 0x6
2025-01-01 09:36:39,423 - DEBUG - Task Done!
2025-01-01 09:36:39,423 - DEBUG - File: LN882H_RAM_BIN.bin
2025-01-01 09:36:39,424 - DEBUG - Size: 37872 Bytes
2025-01-01 09:36:39,424 - DEBUG - Packets: 37
Start program. Wait 10 seconds
send version... wait for:  RAMCODE
b'version\r\n'
b'RAMCODE\r\n'
flash_uid
flash uid:0x433031373839372E30300900C30073FF
Change baudrate 921600
b'baudrate 921600'
Wait 5 seconds for sync
send version... wait for:  RAMCODE
b'version\r\n'
b'RAMCODE\r\n'
Done
startaddr 0x0
pppp
2025-01-01 09:36:54,464 - DEBUG - <<< 0x43
2025-01-01 09:36:54,464 - DEBUG - Packet 0 >>>
2025-01-01 09:36:54,609 - DEBUG - <<< 0x6
2025-01-01 09:36:54,609 - DEBUG - <<< 0x43
2025-01-01 09:36:54,805 - DEBUG - Packet 1 >>>
2025-01-01 09:36:54,812 - DEBUG - <<< ACK
2025-01-01 09:36:55,011 - DEBUG - Packet 2 >>>
2025-01-01 09:36:55,019 - DEBUG - <<< ACK
2025-01-01 09:36:55,215 - DEBUG - Packet 3 >>>
2025-01-01 09:36:55,222 - DEBUG - <<< ACK
2025-01-01 09:36:55,418 - DEBUG - Packet 4 >>>
2025-01-01 09:36:55,425 - DEBUG - <<< ACK
2025-01-01 09:36:55,621 - DEBUG - Packet 5 >>>
2025-01-01 09:36:55,628 - DEBUG - <<< ACK
2025-01-01 09:36:55,824 - DEBUG - Packet 6 >>>
2025-01-01 09:36:55,832 - DEBUG - <<< ACK
2025-01-01 09:36:56,028 - DEBUG - Packet 7 >>>
2025-01-01 09:36:56,036 - DEBUG - <<< ACK
2025-01-01 09:36:56,231 - DEBUG - Packet 8 >>>
2025-01-01 09:36:56,239 - DEBUG - <<< ACK
2025-01-01 09:36:56,434 - DEBUG - Packet 9 >>>
2025-01-01 09:36:56,442 - DEBUG - <<< ACK
2025-01-01 09:36:56,638 - DEBUG - Packet 10 >>>
2025-01-01 09:36:56,645 - DEBUG - <<< ACK
2025-01-01 09:36:56,841 - DEBUG - Packet 11 >>>
2025-01-01 09:36:56,848 - DEBUG - <<< ACK
2025-01-01 09:36:57,044 - DEBUG - Packet 12 >>>
2025-01-01 09:36:57,051 - DEBUG - <<< ACK
2025-01-01 09:36:57,247 - DEBUG - Packet 13 >>>
2025-01-01 09:36:57,254 - DEBUG - <<< ACK
2025-01-01 09:36:57,450 - DEBUG - Packet 14 >>>
2025-01-01 09:36:57,458 - DEBUG - <<< ACK
2025-01-01 09:36:57,654 - DEBUG - Packet 15 >>>
2025-01-01 09:36:57,661 - DEBUG - <<< ACK
2025-01-01 09:36:57,857 - DEBUG - Packet 16 >>>
2025-01-01 09:36:57,864 - DEBUG - <<< ACK
2025-01-01 09:36:58,060 - DEBUG - Packet 17 >>>
2025-01-01 09:36:58,067 - DEBUG - <<< ACK
2025-01-01 09:36:58,263 - DEBUG - Packet 18 >>>
2025-01-01 09:36:58,270 - DEBUG - <<< ACK
2025-01-01 09:36:58,466 - DEBUG - Packet 19 >>>
2025-01-01 09:36:58,474 - DEBUG - <<< ACK
2025-01-01 09:36:58,669 - DEBUG - Packet 20 >>>
2025-01-01 09:36:58,677 - DEBUG - <<< ACK
2025-01-01 09:36:58,872 - DEBUG - Packet 21 >>>
2025-01-01 09:36:58,880 - DEBUG - <<< ACK
2025-01-01 09:36:59,075 - DEBUG - Packet 22 >>>
2025-01-01 09:36:59,083 - DEBUG - <<< ACK
2025-01-01 09:36:59,282 - DEBUG - Packet 23 >>>
2025-01-01 09:36:59,289 - DEBUG - <<< ACK
2025-01-01 09:36:59,485 - DEBUG - Packet 24 >>>
2025-01-01 09:36:59,492 - DEBUG - <<< ACK
2025-01-01 09:36:59,688 - DEBUG - Packet 25 >>>
2025-01-01 09:36:59,696 - DEBUG - <<< ACK
2025-01-01 09:36:59,891 - DEBUG - Packet 26 >>>
2025-01-01 09:36:59,899 - DEBUG - <<< ACK
2025-01-01 09:37:00,095 - DEBUG - Packet 27 >>>
2025-01-01 09:37:00,102 - DEBUG - <<< ACK
2025-01-01 09:37:00,298 - DEBUG - Packet 28 >>>
2025-01-01 09:37:00,305 - DEBUG - <<< ACK
2025-01-01 09:37:00,504 - DEBUG - Packet 29 >>>
2025-01-01 09:37:00,511 - DEBUG - <<< ACK
2025-01-01 09:37:00,707 - DEBUG - Packet 30 >>>
2025-01-01 09:37:00,715 - DEBUG - <<< ACK
2025-01-01 09:37:00,910 - DEBUG - Packet 31 >>>
2025-01-01 09:37:00,918 - DEBUG - <<< ACK
2025-01-01 09:37:01,114 - DEBUG - Packet 32 >>>
2025-01-01 09:37:01,121 - DEBUG - <<< ACK
2025-01-01 09:37:01,317 - DEBUG - Packet 33 >>>
2025-01-01 09:37:01,324 - DEBUG - <<< ACK
2025-01-01 09:37:01,520 - DEBUG - Packet 34 >>>
2025-01-01 09:37:01,527 - DEBUG - <<< ACK
2025-01-01 09:37:01,724 - DEBUG - Packet 35 >>>
2025-01-01 09:37:01,731 - DEBUG - <<< ACK
2025-01-01 09:37:01,927 - DEBUG - Packet 36 >>>
2025-01-01 09:37:01,934 - DEBUG - <<< ACK
2025-01-01 09:37:02,130 - DEBUG - Packet 37 >>>
2025-01-01 09:37:02,137 - DEBUG - <<< ACK
2025-01-01 09:37:02,333 - DEBUG - Packet 38 >>>
2025-01-01 09:37:02,341 - DEBUG - <<< ACK
2025-01-01 09:37:02,537 - DEBUG - Packet 39 >>>
2025-01-01 09:37:02,544 - DEBUG - <<< ACK
2025-01-01 09:37:02,740 - DEBUG - Packet 40 >>>
2025-01-01 09:37:02,747 - DEBUG - <<< ACK
2025-01-01 09:37:02,943 - DEBUG - Packet 41 >>>
2025-01-01 09:37:02,950 - DEBUG - <<< ACK
2025-01-01 09:37:02,950 - DEBUG - EOF
2025-01-01 09:37:02,951 - DEBUG - >>> EOT
2025-01-01 09:37:02,964 - DEBUG - <<< 0x15
2025-01-01 09:37:02,964 - DEBUG - >>> EOT
2025-01-01 09:37:02,966 - DEBUG - <<< 0x6
2025-01-01 09:37:02,966 - DEBUG - <<< 0x43
2025-01-01 09:37:02,966 - DEBUG - Packet End >>>
2025-01-01 09:37:02,970 - DEBUG - <<< 0x6
2025-01-01 09:37:02,970 - DEBUG - Task Done!
2025-01-01 09:37:02,970 - DEBUG - File: OpenLN882H_1.17.662.bin
2025-01-01 09:37:02,970 - DEBUG - Size: 658188 Bytes
2025-01-01 09:37:02,970 - DEBUG - Packets: 41
b'filecount'
b'fc:1'
Change baudrate 115200
b'baudrate 115200'
Wait 5 seconds for sync
send version... wait for:  RAMCODE
b'version\r\n'
b'RAMCODE\r\n'
Done
```


## Build firmware with docker


```
git clone https://github.com/openshwprojects/OpenBK7231T_App.git

git pull --recurse-submodules

docker run --env TARGET_SDKS="OpenLN882H" -it -v "$(pwd)/..":/OpenBK7231T_App  openbk_build 
```