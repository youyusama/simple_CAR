CSOURCES = src/visualization/aigsim_for_vis.c src/model/aiger.c

CPPSOURCES = src/model/CarSolver.cpp src/check/BackwardChecker.cpp src/check/ForwardChecker.cpp src/model/AigerModel.cpp src/model/MainSolver.cpp src/model/InvSolver.cpp src/model/State.cpp src/main.cpp src/model/OverSequence.cpp src/model/OverSequenceNI.cpp src/model/Log.cpp src/visualization/Vis.cpp src/model/OverSequenceForProp.cpp\
	src/sat/minisat/core/Solver.cc src/sat/minisat/utils/Options.cc src/sat/minisat/utils/System.cc
#CSOURCES = aiger.c picosat/picosat.c
#CPPSOURCES = bfschecker.cpp checker.cpp carsolver.cpp mainsolver.cpp model.cpp utility.cpp data_structure.cpp main.cpp \
	glucose/core/Solver.cc glucose/utils/Options.cc glucose/utils/System.cc

OBJS = CarSolver.o BackwardChecker.o ForwardChecker.o AigerModel.o State.o main.o aiger.o aigsim_for_vis.o MainSolver.o InvSolver.o OverSequence.o OverSequenceNI.o Log.o Vis.o OverSequenceForProp.o\
	Solver.o Options.o System.o

# CFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat -I./src/model -I./src/visualization -I./src/check -I./src/sat/cadical/src -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g -fpermissive -DNDEBUG -O3
CFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat -I./src/model -I./src/visualization -I./src/check -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g

CPPFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat -I./src/model -I./src/visualization -I./src/check -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g -fpermissive

# LFLAG = -g -lz -lpthread -L./ -DNDEBUG
LFLAG = -g -lz -lpthread -L./ 

GCC = gcc

GXX = g++

simplecar: $(CSOURCES) $(CPPSOURCES)
	$(GCC) $(CFLAG) $(CSOURCES)
	$(GCC) $(CPPFLAG) -std=c++11 $(CPPSOURCES)
	$(GXX) -o simplecar_minisat $(OBJS) $(LFLAG)
	rm *.o

.PHONY: simplecar

clean: 
	
	rm simplecar
	

