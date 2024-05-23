#!/bin/bash
busybox devmem 0x0c303018 w 0xc458
busybox devmem 0x0c303010 w 0xc400
busybox devmem 0x0c303018
busybox devmem 0x0c303010

modprobe can
modprobe can_raw
modprobe mttcan

ip link set can0 up type can bitrate 1000000 dbitrate 2000000 berr-reporting on fd on