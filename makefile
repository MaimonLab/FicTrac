#Makefile for FicTrac
#Modified for compatibility with the MCC USB3100
#
#  Ver 1.1

#Declarations for mcclibhid
CC=gcc
SOURCESM= ./library/pmd.c ./library/usb-3100.c
HEADERS= ./library/pmd.h ./library/usb-3100.h
OBJECTSM= $(SOURCESM:.c=.o)
CFLAGS= -g -Wall -fPIC -O
TARGETSM= libmcchid.so libmcchid.a


#Declarations for FicTrac
CC2=g++
CFLAGSF=-DPGR_CAMERA -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -I"./library" -I"/usr/include/cairomm-1.0" -O3 -Wall -c -fmessage-length=0 -std=c++0x -Wno-unused-function `pkg-config --cflags cairomm-1.0` -MMD
OBJECTSF=$(SOURCESF:.cpp=.o)
DEPENDS=$(SOURCESF:.cpp=.d)
LDLIBS=-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lpthread -lavformat -lavcodec -lavutil -lswscale -lnlopt -lcairo -lcairomm-1.0 -lsigc-2.0 -lrt -lflycapture
EXECUTABLE=FicTrac
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
    ./library/readwrite.cpp \
    ./library/RectilinearCameraModel.cpp \
    ./library/Remapper.cpp \
    ./library/PGRSource.cpp \
    ./library/serial.cpp \
    ./library/Utils.cpp \
    ./library/VsDraw.cpp



#MAKE RULES

all: $(SOURCES) $(EXECUTABLE)

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

clean: ; rm -f $(DEPENDS) $(OBJECTSF) $(EXECUTABLE)

