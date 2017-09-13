#!/bin/bash
if [ $EUID -ne 0 ];
then
   echo `date +"%Y-%m-%d %T "`"This script must be run as root. use sudo su" 1>&2
   exit 1
fi

apt-get update            
apt-get install subversion
apt-get install libusb-1.0-0-dev libusb-1.0-0 libcurl4-openssl-dev libncurses5-dev libudev-dev
apt-get install libtool autoconf automake

chmod a+x ./autogen.sh
./autogen.sh
./configure --enable-ltctech --enable-coinflex
# ./configure --enable-ltctech					# Blocked by HKS 2016.07.05
# ./configure  --enable-coinflex					# Blocked by HKS 2016.07.05

