#!/bin/sh

#make distclean
./autogen.sh

#ROOTFS_DIR=$1
ROOTFS_DIR=/home/public/duanh/src/rootfs
#MAKE_JOBS=$2
MAKE_JOBS=8
ROOTFS_DIR_G19=$3

LDFLAGS="-L${ROOTFS_DIR}/lib -linno -lim_lib -lim_drv " \
CFLAGS="-I${ROOTFS_DIR}/include -Wall " \
./configure --prefix=${ROOTFS_DIR} \
--enable-coinflex --without-curses --host=arm-xilinx-linux-gnueabi --build=x86_64-pc-linux-gnu #--target=arm

make -j${MAKE_JOBS}
arm-xilinx-linux-gnueabi-strip sgminer
cp sgminer innominer_T3

