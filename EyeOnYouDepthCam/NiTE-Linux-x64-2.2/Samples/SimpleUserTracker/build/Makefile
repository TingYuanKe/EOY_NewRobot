include ../CommonDefs.mak

BIN_DIR = Bin

INC_DIRS = \
	../Common \
	../../Include \
	$(OPENNI2_INCLUDE) \
	home/Desktop/EOY_NewRobot/EyeOnYouDepthCam/NiTE-Linux-x64-2.2/Include \
	/usr/include/openni2 \
SRC_FILES = *.cpp

LIB_DIRS += $(OPENNI2_REDIST)
LIB_DIRS += home/Desktop/EOY_NewRobot/EyeOnYouDepthCam/NiTE-Linux-x64-2.2/Include \

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = SimpleUserTracker

CFLAGS += -Wall g++

include ../CommonCppMakefile
