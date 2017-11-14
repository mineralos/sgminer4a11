#!/bin/sh

#make distclean
./autogen.sh

SGMINER_DIR=$PWD/sgminer_lib
MAKE_JOBS=4

mkdir ${SGMINER_DIR}
cd ../curl
./buildconf
./configure --prefix=${SGMINER_DIR} --host=arm-xilinx-linux-gnueabi --build=x86_64-pc-linux-gnu --enable-shared=yes --enable-static=no --enable-silent-rules --disable-debug --enable-optimize --enable-warnings --disable-curldebug --disable-symbol-hiding --enable-http --disable-ftp --disable-file --disable-ldap --disable-ldaps --disable-rtsp --disable-proxy --disable-dict --enable-telnet --disable-tftp --disable-pop3 --disable-imap --disable-smb --disable-smtp --disable-gopher --disable-manual --disable-libcurl-option --disable-ipv6 --disable-versioned-symbols --disable-threaded-resolver --disable-verbose --disable-sspi --disable-crypto-auth --disable-ntlm-wb --disable-tls-srp --enable-unix-sockets --disable-cookies --disable-soname-bump --without-zlib --without-winssl --without-darwinssl --without-ssl --without-gnutls --without-polarssl --without-mbedtls --without-cyassl --without-nss --without-axtls --without-ca-bundle --without-ca-path --without-ca-fallback --without-libpsl --without-libmetalink --without-libssh2 --without-librtmp --without-winidn --without-libidn2 --without-nghttp2 --without-zsh-functions-dir

make -j${MAKE_JOBS}
make install

cd ../sgminer
LDFLAGS=-L${SGMINER_DIR}/lib \
CFLAGS=-I${SGMINER_DIR}/include \
./configure --prefix=${SGMINER_DIR} \
--enable-coinflex --without-curses --host=arm-xilinx-linux-gnueabi --build=x86_64-pc-linux-gnu #--target=arm

make -j${MAKE_JOBS}

#cp sgminer /home/public/update/sgminer_a7.$1
#chmod 777 /home/public/update/sgminer_a7.$1

rm -rf ${SGMINER_DIR}
