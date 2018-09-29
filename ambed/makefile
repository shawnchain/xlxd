CC=g++
C=gcc
CFLAGS=-c -std=c++11 -pthread -DNO_FTDI_LIB -g -ggdb
LDFLAGS=-std=c++11 -pthread 
SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=ambed

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS) .ftd2xx_drv
#	$(CC) $(LDFLAGS) $(OBJECTS) -lftd2xx -Wl,-rpath,/usr/local/lib -o $@
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

.ftd2xx_drv:
	@echo "compile ftd2xx_drv stub"
#	$(CC) $(CFLAGS) ftd2xx_drv.cpp -o ftd2xx_drv.o

clean:
	$(RM) $(EXECUTABLE) *.o

install:
	mkdir -p /ambed
	cp $(EXECUTABLE) /ambed/
	cp ./run /ambed/
