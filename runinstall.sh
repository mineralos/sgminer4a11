#!/bin/sh

ROOTFS_DIR=$1
ROOTFS_DIR_G19=$2

# 安装sgminer及其库(删除sgminer)
make install
rm -rf ${ROOTFS_DIR}/bin/sgminer

cp innominer_* ${ROOTFS_DIR}/bin/
mv innominer_* ${ROOTFS_DIR_G19}/bin/
