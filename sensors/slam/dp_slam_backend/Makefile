CXX=g++
OPTS=-O3
CFLAGS=$(OPTS) -Wall -lGL -lGLU -lglut -lGLEW -std=c++1y
LIBS=

ifeq ($(OS),Windows_NT)
	LIBS+=-lWs2_32
else
	ifeq ($(shell uname),Darwin)
		LIBS+=-framework IOKit -framework CoreFoundation 
	endif
endif

all: dp_slam_backend

dp_slam_backend: dp_slam_backend.cpp
	$(CXX) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm -rf dp_slam_backend dp_slam_backend.exe
