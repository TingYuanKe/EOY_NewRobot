include ../BuildSystem/CommonDefs.mak

BIN_DIR = ../Bin

INC_DIRS = \
	../../Include \
	../GL \
	$(OPENNI2_INCLUDE)

SRC_FILES = *.cpp

ifeq ("$(OSTYPE)","Darwin")
	CFLAGS += MACOS
	LDFLAGS += -framework OpenGL -framework GLUT
else
	CFLAGS += -DUNIX -DGLX_GLXEXT_LEGACY
	USED_LIBS += glut GL
endif

LIB_DIRS += $(OPENNI2_REDIST)

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = UserViewer

CFLAGS += -Wall

# TingYuan: compile with c++ to support <thread.h>
CFLAGS += -std=c++11

# ChihHsiang: compile with c++ to support <pthread.h>
LDFLAGS += -lpthread


include ../BuildSystem/CommonCppMakefile
	
