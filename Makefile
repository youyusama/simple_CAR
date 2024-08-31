CSOURCES = \
src/model/aiger.c

CPPSOURCES = \
src/model/AigerModel.cpp\
src/model/Log.cpp\
src/check/BackwardChecker.cpp\
src/check/ForwardChecker.cpp\
src/check/BMC.cpp\
src/model/MainSolver.cpp\
src/model/State.cpp\
src/main.cpp\
src/model/Branching.cpp\
src/model/OverSequenceSet.cpp

OBJS = \
aiger.o\
AigerModel.o \
Log.o\
BackwardChecker.o\
ForwardChecker.o\
BMC.o\
MainSolver.o\
State.o\
main.o\
Branching.o\
OverSequenceSet.o

ifdef CADICAL
	CPPSOURCES += src/model/CadicalSolver.cpp
	OBJS += CadicalSolver.o
	OBJS += src/sat/cadical/build/libcadical.a
else
	CPPSOURCES += src/model/MinisatSolver.cpp
	CPPSOURCES += src/sat/minisat/core/Solver.cc
	CPPSOURCES += src/sat/minisat/simp/SimpSolver.cc
	CPPSOURCES += src/sat/minisat/utils/Options.cc
	CPPSOURCES += src/sat/minisat/utils/System.cc
	OBJS += MinisatSolver.o
	OBJS += SimpSolver.o
	OBJS += Solver.o
	OBJS += Options.o
	OBJS += System.o
endif

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
	

