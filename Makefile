CSOURCES = \
src/model/aiger.c

CPPSOURCES = \
src/model/Model.cpp\
src/model/Log.cpp\
src/check/BackwardChecker.cpp\
src/check/ForwardChecker.cpp\
src/check/BMC.cpp\
src/model/SATSolver.cpp\
src/model/IncrCheckerHelpers.cpp\
src/main.cpp

OBJS = \
aiger.o\
Model.o \
Log.o\
BackwardChecker.o\
ForwardChecker.o\
BMC.o\
SATSolver.o\
IncrCheckerHelpers.o\
main.o

ifdef CADICAL
	CPPSOURCES += src/model/CadicalSolver.cpp
	OBJS += CadicalSolver.o
	OBJS += src/sat/cadical/build/libcadical.a
endif

CPPSOURCES += src/model/MinisatSolver.cpp
CPPSOURCES += src/sat/minisat/core/Solver.cc
CPPSOURCES += src/sat/minisat/simp/SimpSolver.cc
CPPSOURCES += src/sat/minisat/utils/Options.cc
CPPSOURCES += src/sat/minisat/utils/System.cc
OBJS += MinisatSolver.o
OBJS += Solver.o
OBJS += SimpSolver.o
OBJS += Options.o
OBJS += System.o


CFLAG = -c -g

CPPFLAG = -I./src/sat -I./src/model -I./src/check -c -g -fpermissive

ifndef DEBUG
	CFLAG += -O3 -s -DNDEBUG
	CPPFLAG += -O3 -s -DNDEBUG
endif

LFLAG = -g -lz -lpthread -L./ -static -flto

ifdef CADICAL
	CPPFLAG += -DCADICAL
endif

GCC = gcc

GXX = g++

simplecar: $(CSOURCES) $(CPPSOURCES)
	$(GCC) $(CFLAG) $(CSOURCES)
	$(GCC) $(CPPFLAG) -std=c++11 $(CPPSOURCES)
	$(GXX) -o simplecar $(OBJS) $(LFLAG)
	rm *.o

.PHONY: simplecar

clean: 
	rm simplecar
	rm *.o
	

