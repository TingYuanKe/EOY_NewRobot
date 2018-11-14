CC = g++

BIN_DIR = Bin

INC_DIRS = \
	~/Desktop/EOY_NewRobot/EyeOnYouDepthCam/doc/NiTE-Linux-x64-2.2-test/Include \
	/usr/include/openni2
SRC_FILES = *.cpp

ifeq ("$(OSTYPE)","Darwin")
	CFLAGS += -DMACOS
	LDFLAGS += -framework OpenGL -framework GLUT
else
	CFLAGS += -DUNIX -DGLX_GLXEXT_LEGACY
	USED_LIBS += glut GL
endif

LIB_DIRS += $(OPENNI2_REDIST)

USED_LIBS += OpenNI2 NiTE2

EXE_NAME = UserViewer

CFLAGS += -g -Wall -ansi


UserViewr: main.o Viewer.o	 
	${CC} main.o Viewer.o ${CFLAGS} -I${INC_DIRS} ${LIB_DIRS} -o $@
%.o: %.cpp %.h                     
	${CC} $< -I${CFLAGS} ${INC_DIRS} ${LIB_DIR} -lpthread -c   

.PHONY: clean

clean:
	@rm -rf *.o 

