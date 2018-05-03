#!/bin/bash
ARCH=arm CROSS_COMPILE=arm-ais-linux-gnueabihf- make distclean
ARCH=arm CROSS_COMPILE=arm-ais-linux-gnueabihf- make val100_defconfig 
ARCH=arm CROSS_COMPILE=arm-ais-linux-gnueabihf- make -j8
exit 0
