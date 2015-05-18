#!/bin/bash

PATH=$PATH:/arago-2011.09/armv5te/bin \
	make \
		ARCH=arm CROSS_COMPILE=arm-arago-linux-gnueabi- \
		-C ../linux-3.3-psp03.22.00.06.sdk \
	SUBDIRS=$PWD clean
