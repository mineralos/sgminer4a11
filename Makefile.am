
ACLOCAL_AMFLAGS = -I m4

JANSSON_CPPFLAGS= -I$(top_builddir)/compat/jansson-2.9/src -I$(top_srcdir)/compat/jansson-2.9/src

EXTRA_DIST	= example.conf linux-usb-sgminer \
		  api-example.php miner.php	\
		  API.class API.java api-example.c windows-build.txt \
		  bitstreams/README API-README FPGA-README \
		  bitforce-firmware-flash.c hexdump.c ASIC-README \
		  01-sgminer.rules

SUBDIRS		= lib compat ccan sph

sgminer_CPPFLAGS = $(PTHREAD_FLAGS) -fno-strict-aliasing $(JANSSON_CPPFLAGS) $(USBUTILS_CPPFLAGS)

bin_PROGRAMS	= sgminer

sgminer_LDFLAGS	= $(PTHREAD_FLAGS)
sgminer_LDADD	= $(DLOPEN_FLAGS) @LIBCURL_LIBS@ @JANSSON_LIBS@ @PTHREAD_LIBS@ \
		  @NCURSES_LIBS@ @PDCURSES_LIBS@ @WS2_LIBS@ \
		  @LIBUSB_LIBS@ @MM_LIBS@ @RT_LIBS@ \
		  @MATH_LIBS@ lib/libgnu.a ccan/libccan.a sph/libsph.a

sgminer_CPPFLAGS += -I$(top_builddir)/lib -I$(top_srcdir)/lib

if !HAVE_WINDOWS
sgminer_CPPFLAGS += @LIBCURL_CFLAGS@
endif

# common sources
sgminer_SOURCES := sgminer.c

sgminer_SOURCES	+= elist.h miner.h compat.h bench_block.h	\
		   util.c util.h uthash.h logging.h		\
		   sha2.c sha2.h api.c

sgminer_SOURCES	+= logging.c

sgminer_SOURCES	+= klist.h klist.c

sgminer_SOURCES	+= noncedup.c

sgminer_SOURCES += scrypt.c scrypt.h

#sgminer_SOURCES += darkcoin.c darkcoin.h
#sgminer_SOURCES += marucoin.c marucoin.h

# Device drivers
if HAS_LTCTECH
sgminer_SOURCES += driver-ltctech.c
# bitstreamsdir = $(bindir)/bitstreams						# Blocked by HKS 2016.07.05
# dist_bitstreams_DATA = $(top_srcdir)/bitstreams/README	# Blocked by HKS 2016.07.05
endif

if HAS_COINFLEX
sgminer_SOURCES += driver-coinflex.c
sgminer_SOURCES += spi-context.c spi-context.h
sgminer_SOURCES += A1-common.h
sgminer_SOURCES += A1-board-selector.h
sgminer_SOURCES += A1-board-selector-CCD.c A1-board-selector-CCR.c
sgminer_SOURCES += A1-trimpot-mcp4x.h A1-trimpot-mcp4x.c
sgminer_SOURCES += i2c-context.c i2c-context.h

sgminer_SOURCES += asic_inno.c
sgminer_SOURCES += asic_inno_cmd.c
sgminer_SOURCES += asic_inno_clock.c
sgminer_SOURCES += asic_inno_gpio.c
#sgminer_SOURCES += scrypt.c
# bitstreamsdir = $(bindir)/bitstreams						# Blocked by HKS 2016.07.05
# dist_bitstreams_DATA = $(top_srcdir)/bitstreams/README	# Blocked by HKS 2016.07.05
endif

