#!/bin/sh


if [ -f Makefile ]; then
    make clean
    make distclean
fi

rm -rf sgminer
rm -rf innominer*

# compat 清理不干净
#git co compat

