CSOURCES = src/visualization/aigsim_for_vis.c src/model/aiger.c

CPPSOURCES = src/model/CarSolver.cpp src/model/CarSolver_cadical.cpp src/check/BackwardChecker.cpp src/check/ForwardChecker.cpp src/check/CleanCARCheckerF.cpp src/model/AigerModel.cpp src/model/MainSolver.cpp src/model/InvSolver.cpp src/model/State.cpp src/main.cpp src/model/OverSequence.cpp src/model/Log.cpp src/visualization/Vis.cpp src/model/OverSequenceForProp.cpp\
	src/sat/minisat/core/Solver.cc src/sat/minisat/utils/Options.cc src/sat/minisat/utils/System.cc
#CSOURCES = aiger.c picosat/picosat.c
#CPPSOURCES = bfschecker.cpp checker.cpp carsolver.cpp mainsolver.cpp model.cpp utility.cpp data_structure.cpp main.cpp \
	glucose/core/Solver.cc glucose/utils/Options.cc glucose/utils/System.cc

OBJS = CarSolver.o CarSolver_cadical.o BackwardChecker.o ForwardChecker.o CleanCARCheckerF.o AigerModel.o State.o main.o aiger.o aigsim_for_vis.o MainSolver.o InvSolver.o OverSequence.o Log.o Vis.o OverSequenceForProp.o\
	Solver.o Options.o System.o

# CFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat -I./src/model -I./src/check -I./src/sat/cadical/src -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g -fpermissive -DNDEBUG -DQUIET -O3 -DCADICAL
CFLAG = -I../ -I./src/sat/minisat/core -I./src/sat/minisat -I./src/model -I./src/visualization -I./src/check -I./src/sat/cadical/src -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g -fpermissive -DNDEBUG -DQUIET -O3
#CFLAG = -I../ -I./glucose -D__STDC_LIMIT_MACROS -D __STDC_FORMAT_MACROS -c -g 

LFLAG = -g -lz -lpthread -L./src/sat/cadical/build -L./ -lcadical -DNDEBUG -DQUIET
# LFLAG = -g -lz -lpthread -L./src/sat/cadical/build -L./ -DNDEBUG -DQUIET

GCC = gcc

GXX = g++

simplecar: $(CSOURCES) $(CPPSOURCES)
	$(GCC) $(CFLAG) $(CSOURCES)
	$(GCC) $(CFLAG) -std=c++11 $(CPPSOURCES)
# $(GXX) -o simplecar_cadical $(OBJS) $(LFLAG)
	$(GXX) -o simplecar_minisat $(OBJS) $(LFLAG)
	rm *.o

.PHONY: simplecar

clean: 
	
	rm simplecar
	

