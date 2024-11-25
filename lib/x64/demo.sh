#! /bin/sh
ar x libGuideUSB2LiveStream.a
ar x libusb-1.0.a
ar crU libGuideUSB2LiveStream.a *.o
ranlib libGuideUSB2LiveStream.a
rm *.o


