#Makefile for FicTrac
#Modified for compatibility with the MCC USB3100
#
#  Ver 1.1

#Declarations for fmfwrapper
FMF_SOURCES = ./fmfwrapper/handle_error.cpp \
              ./fmfwrapper/pyboostcvconverter.cpp \
              ./fmfwrapper/fmfwrapper.cpp
FMF_OBJS = handle_error.o fmfwrapper.o pyboostcvconverter.o
BOOST_PYTHON_LIB = "/usr/local/lib/libboost_python.a"
FMF_INC_DIRS = -I"/usr/local/include" -I"/usr/include/python2.7" -I"./fmfwrapper"

#Declarations for mcclibhid
CC=gcc
SOURCESM= ./library/pmd.c ./library/usb-3100.c
HEADERS= ./library/pmd.h ./library/usb-3100.h
OBJECTSM= $(SOURCESM:.c=.o)
CFLAGS= -g -Wall -fPIC -O
TARGETSM= libmcchid.so libmcchid.a


#Declarations for FicTrac
CC2=g++
CFLAGSF=-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -I"./library" -I"/usr/include/cairomm-1.0" $(FMF_INC_DIRS) -O3 -Wall -c -fmessage-length=0 -std=c++0x -Wno-unused-function `pkg-config --cflags cairomm-1.0` -MMD
OBJECTSF=$(SOURCESF:.cpp=.o)
DEPENDS=$(SOURCESF:.cpp=.d)
LDLIBS=-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lpthread -lavformat -lavcodec -lavutil -lswscale -lnlopt -lcairo -lcairomm-1.0 -lsigc-2.0 -lrt -lpython2.7
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
    ./library/PGRSource.cpp \
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

libmcchid.so: $(OBJECTSM)
#	$(CC) -O -shared -Wall $(OBJECTSM) -o $@
	$(CC) -shared -Wl,-soname,$@ -o $@ $(OBJECTSM) -lc -lm

libmcchid.a: $(OBJECTSM)
	ar -r libmcchid.a $(OBJECTSM)
	ranlib libmcchid.a

fmfwrapper.o: $(FMF_OBJS) $(BOOST_PYTHON_LIB) 
	$(CC2) $(CFLAGSF) $< -o $@ 

$(EXECUTABLE): $(OBJECTSF)
	$(CC2) -o $@ $(OBJECTSF) $(LDFLAGS) $(LDLIBS) -g -Wall -I. -lmcchid -L. -lm -L/usr/local/lib -lhid -lusb 

.cpp.o:
	$(CC2) $(CFLAGSF) $< -o $@

clean: ; rm -f $(DEPENDS) $(OBJECTSF) $(FMF_OBJS) $(EXECUTABLE)

