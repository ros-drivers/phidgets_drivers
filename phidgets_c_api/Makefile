all:build

TARBALL      = build/libphidget_2.1.8.20120514.tar.gz
TARBALL_URL  = http://www.phidgets.com/downloads/libraries/libphidget_2.1.8.20120514.tar.gz
MD5SUM_FILE  = libphidget_2.1.8.20120514.md5sum
UNPACK_CMD   = tar xzf
SOURCE_DIR   = build/libphidget-2.1.8.20120514
SOURCE_DIR_G = $(shell rospack find phidgets_c_api)/$(SOURCE_DIR)

include $(shell rospack find mk)/download_unpack_build.mk

build: configured
	make -C $(SOURCE_DIR)
	make install -C $(SOURCE_DIR)
	mkdir -p include/phidgets_c_api
	cp $(SOURCE_DIR)/include/phidget21.h include/phidgets_c_api
	mkdir -p lib
	cp $(SOURCE_DIR)/lib/libphidget21.so.0.0.0 lib/libphidget21.so
	cp $(SOURCE_DIR)/lib/libphidget21.so.0.0.0 lib/libphidget21.so.0
	cp $(SOURCE_DIR)/lib/libphidget21.a        lib/libphidget21.a
	cp $(SOURCE_DIR)/lib/libphidget21.la       lib/libphidget21.la

configured: $(SOURCE_DIR)/unpacked
	cd $(SOURCE_DIR) && ./configure --disable-jni --prefix=$(SOURCE_DIR_G)
	touch configured

clean:
	rm -rf $(SOURCE_DIR)

wipe: clean
	rm -rf build include lib
	rm -f configured  
