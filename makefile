CC=g++

CFLAGS=-D__STDC_LIMIT_MACROS -D__STDC_CONSTANT_MACROS -I"./library" -O3 -Wall -c -fmessage-length=0 -std=c++0x -Wno-unused-function `pkg-config --cflags cairomm-1.0` -MMD

LDLIBS=-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_video -lpthread -lnlopt -lcairo -lcairomm-1.0 -lsigc-2.0 -lrt

SOURCES=FicTrac.cpp \
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

OBJECTS=$(SOURCES:.cpp=.o)

DEPENDS=$(SOURCES:.cpp=.d)

EXECUTABLE=FicTrac

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) -o $@ $(OBJECTS) $(LDFLAGS) $(LDLIBS)

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

clean: ; rm -f $(DEPENDS) $(OBJECTS) $(EXECUTABLE)

