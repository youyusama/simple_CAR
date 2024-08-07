CSOURCES = \
src/model/aiger.c

CPPSOURCES = \
src/model/CarSolver.cpp\
src/check/BackwardChecker.cpp\
src/check/ForwardChecker.cpp\
src/model/AigerModel.cpp\
src/model/MainSolver.cpp\
src/model/InvSolver.cpp\
src/model/State.cpp\
src/main.cpp\
src/model/Branching.cpp\
src/model/OverSequenceSet.cpp\
src/model/Log.cpp\
src/sat/minisat/core/Solver.cc\
src/sat/minisat/simp/SimpSolver.cc\
src/sat/minisat/utils/Options.cc\
src/sat/minisat/utils/System.cc

OBJS = \
CarSolver.o \
BackwardChecker.o \
ForwardChecker.o \
AigerModel.o \
State.o \
main.o \
aiger.o \
MainSolver.o \
InvSolver.o \
Branching.o\
OverSequenceSet.o \
Log.o \
SimpSolver.o\
Solver.o \
Options.o \
System.o

# CFLAG = -c -g
CFLAG = -c -g -O3 -s -DNDEBUG

# CPPFLAG = -I./src/sat -I./src/model -I./src/check -c -g -fpermissive
CPPFLAG = -I./src/sat -I./src/model -I./src/check -c -g -fpermissive -O3 -s -DNDEBUG

LFLAG = -g -lz -lpthread -L./ -static -flto

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
	

