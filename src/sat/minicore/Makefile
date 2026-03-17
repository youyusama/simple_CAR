CXX ?= g++
CXXFLAGS ?= -std=c++17 -Wall -Wextra
TARGET ?= minicore

ifeq ($(DEBUG),1)
	CXXFLAGS += -g -O0 -ggdb
else
	CXXFLAGS += -O3 -DNDEBUG
endif

SRCS := src/main.cpp src/dimacs.cpp src/solver.cpp
HDRS := src/dimacs.h src/solver.h src/solver_types.h src/utils.hpp
OBJS := $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $^ -o $@

%.o: %.cpp $(HDRS)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
