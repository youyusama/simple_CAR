CSOURCES = \
src/visualization/aigsim_for_vis.c\
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
src/visualization/Vis.cpp\
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
aigsim_for_vis.o \
MainSolver.o \
InvSolver.o \
Branching.o\
OverSequenceSet.o \
Log.o \
Vis.o \
SimpSolver.o\
Solver.o \
Options.o \
System.o

CFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat/simp -I./src/sat/minisat -I./src/model -I./src/visualization -I./src/check -I./src/sat/cadical/src\
 -D__STDC_LIMIT_MACROS \
 -D __STDC_FORMAT_MACROS \
 -c \
 -g \
 -DNDEBUG \
 -O3

CPPFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat/simp -I./src/sat -I./src/model -I./src/visualization -I./src/check \
-D__STDC_LIMIT_MACROS \
-D __STDC_FORMAT_MACROS \
-c \
-g \
-fpermissive \
-DNDEBUG \
-O3

LFLAG = -g -lz -lpthread -L./ \
-DNDEBUG -static

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
	

