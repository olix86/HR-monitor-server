csrc = $(wildcard src/*.c) 

cppsrc = $(wildcard src/*.cpp)

obj = $(csrc:.c=.o) $(cppsrc:.cpp=.o)

CXX=g++
CXXFLAGS= -I inc/
#LDFLAGS =


server: $(obj)
	$(CXX) -o $@ $^ $(CXXFLAGS) 
