all:build

TARBALL      = build/libphidget_2.1.8.20120514.tar.gz
TARBALL_URL  = http://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20120514.tar.gz
MD5SUM_FILE  = libphidget_2.1.8.20120514.md5sum
UNPACK_CMD   = tar xzf
SOURCE_DIR   = build/libphidget-2.1.8.20120514
SOURCE_DIR_G = $(shell rospack find phidgets_api)/$(SOURCE_DIR)

include $(shell rospack find mk)/download_unpack_build.mk

build: configured
	make -C $(SOURCE_DIR)
	make install -C $(SOURCE_DIR)
	mkdir -p include/phidgets_api
	cp $(SOURCE_DIR)/include/phidget21.h include/phidgets_api
	mkdir -p lib
	cp $(SOURCE_DIR)/lib/libphidget21.so lib/

configured: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --disable-jni --prefix=$(SOURCE_DIR_G)
	touch configured

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build
	rm configured  
