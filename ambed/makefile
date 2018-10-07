CC      = gcc
CXX     = g++
CFLAGS  = -g -O3 -Wall -c -std=c++11 -pthread -DNO_FTDI_LIB
LDFLAGS = -g -O3 -Wall -std=c++11
LIBS    = -lpthread
LDFLAGS = -g

SOURCES=$(wildcard *.cpp)
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=ambed

all: $(SOURCES) $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(LDFLAGS) $(LIBS) $(OBJECTS) -o $@

.cpp.o:
	$(CXX) -c $(CFLAGS) $< -o $@

clean:
	$(RM) $(EXECUTABLE) *.o

install:
	mkdir -p /ambed
	cp $(EXECUTABLE) /ambed/
	cp ./run /ambed/
