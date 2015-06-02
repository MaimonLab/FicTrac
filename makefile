#Makefile for modified FicTrac
#
#By Pablo 7/3/14
#  Ver 1.0
#
#This makefile should compile and create a FicTrac executable 
#compatible with the MCC USB3100 device with four
#analog outputs. Mcclibhid should already be installed
#in the Ubuntu machine you intend to use.

#mcclibhid string declarations
CC=gcc
SOURCESM= pmd.c usb-3100.c
HEADERS= pmd.h usb-3100.h
OBJECTSM= $(SOURCESM:.c=.o) 
CFLAGS= -g -Wall -fPIC -O
TARGETSM= libmcchid.so libmcchid.a


#Fictrac String declarations
CC2=g++
SOURCESF=FicTrac.cpp \
    ./library/AVWriter.cpp \
    ./library/CameraModel.cpp \
    ./library/CameraRemap.cpp \
    ./library/CmPoint.cpp \
    ./library/CVSource.cpp \
    ./library/EquiAreaCameraModel.cpp \
    ./library/FisheyeCameraModel.cpp \
    ./library/ImgSource.cpp \
    ./library/Maths.cpp \
    ./library/NLoptFunc.cpp \
    ./library/PGRSource.cpp \
    ./library/readwrite.cpp \
    ./library/RectilinearCameraModel.cpp \
    ./library/Remapper.cpp \
    ./library/serial.cpp \
    ./library/Utils.cpp \
    ./library/VsDraw.cpp

CFLAGSF=-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -I"./library" -I"/usr/include/cairomm-1.0" -O3 -Wall -c -fmessage-length=0 -std=c++0x -Wno-unused-function `pkg-config --cflags cairomm-1.0` -MMD
OBJECTSF=$(SOURCESF:.cpp=.o)
DEPENDS=$(SOURCESF:.cpp=.d)
LDLIBS=-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lpthread -lavformat -lavcodec -lavutil -lswscale -lnlopt -lcairo -lcairomm-1.0 -lsigc-2.0 -lrt
EXECUTABLE=FicTrac


#MAKE RULES

all: $(TARGETSM) $(SOURCESF) $(EXECUTABLE)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@

libmcchid.so: $(OBJECTSM)
#	$(CC) -O -shared -Wall $(OBJECTSM) -o $@
	$(CC) -shared -Wl,-soname,$@ -o $@ $(OBJECTSM) -lc -lm

libmcchid.a: $(OBJECTSM)
	ar -r libmcchid.a $(OBJECTSM)
	ranlib libmcchid.a
	
$(EXECUTABLE): $(OBJECTSF)
	$(CC2) -o $@ $(OBJECTSF) $(LDFLAGS) $(LDLIBS) -g -Wall -I. -lmcchid -L. -lm -L/usr/local/lib -lhid -lusb 

.cpp.o:
	$(CC2) $(CFLAGSF) $< -o $@


