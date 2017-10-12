#!/bin/sh

ROOTFS_DIR=$1

# 安装sgminer及其库(删除sgminer)
make install
rm -rf ${ROOTFS_DIR}/bin/sgminer

cp innominer_* ${ROOTFS_DIR}/bin/

