#Makefile for FicTrac
#Modified for compatibility with the MCC USB3100
#
#  Ver 1.1

#Declarations for mcclibhid
CC=gcc
CFLAGS= -g -Wall -fPIC -O

#Declarations for FicTrac
CC2=g++
CFLAGSF=-DPGR_CAMERA -D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -Wall -g -c -O3 -fmessage-length=0 -std=c++0x -Wno-unused-function `pkg-config --cflags cairomm-1.0` -MMD
# used to have cflagsf -O3
LDFLAGS=-g -Wall 

OBJECTSF=$(SOURCESF:.cpp=.o)
DEPENDS=$(SOURCESF:.cpp=.d)

LDLIBS= -lm \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \
        -lopencv_video \
        -lopencv_videoio \
        -lopencv_imgcodecs \
        -lpthread \
        -lavformat \
        -lavcodec \
        -lavutil \
        -lswscale \
        -lnlopt \
        -lcairo \
        -lcairomm-1.0 \
        -lsigc-2.0 \
        -lrt \
        -lflycapture \
        -lmccusb \
        -lhidapi-libusb \
        -lusb-1.0 

LDDIRS= -L/usr/local/lib \
        -L/usr/lib

INCDIRS=-I"./library" \
        -I"/usr/local/include/opencv2" \
        -I"/usr/local/include/libusb" \
        -I"/usr/local/include/hidapi" \
        -I"/usr/local/include/flycapture" \
        -I"/usr/include/cairomm-1.0"

EXECUTABLE=FicTrac
SOURCESF=FicTrac.cpp \
    ./library/AVWriter.cpp \
    ./library/CameraModel.cpp \
    ./library/CameraRemap.cpp \
    ./library/CmPoint.cpp \
    ./library/CVSource.cpp \
    ./library/EquiAreaCameraModel.cpp \
    ./library/FisheyeCameraModel.cpp \
    ./library/PGRSource.cpp \
    ./library/ImgSource.cpp \
    ./library/Maths.cpp \
    ./library/NLoptFunc.cpp \
    ./library/readwrite.cpp \
    ./library/RectilinearCameraModel.cpp \
    ./library/Remapper.cpp \
    ./library/serial.cpp \
    ./library/Utils.cpp \
    ./library/VsDraw.cpp



#MAKE RULES

all: $(SOURCES) $(EXECUTABLE)

%.d: %.c
	set -e; $(CC) -I. -M $(CPPFLAGS) $< \
	| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
	[ -s $@ ] || rm -f $@

$(EXECUTABLE): $(OBJECTSF)
	$(CC2) -o $@ $(OBJECTSF) $(LDFLAGS) -I. -L. $(LDDIRS) $(LDLIBS) 

.cpp.o:
	$(CC2) $(CFLAGSF) $(INCDIRS) $< -o $@

clean: ; rm -f $(DEPENDS) $(OBJECTSF) $(EXECUTABLE)

