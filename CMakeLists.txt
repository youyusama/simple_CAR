cmake_minimum_required(VERSION 3.10)

project(simpleCAR VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_SANITIZERS "Enable Address/UB Sanitizers and extra debug flags" OFF)


# debug
set(CMAKE_C_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -fpermissive")

if(ENABLE_SANITIZERS)
    message(STATUS "Sanitizers enabled for Debug builds")
    set(SANITIZER_FLAGS "-O0 -fno-omit-frame-pointer -fsanitize=address,undefined -fno-strict-aliasing")

    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} ${SANITIZER_FLAGS}")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${SANITIZER_FLAGS}")
else()
    message(STATUS "Sanitizers disabled for Debug builds")
endif()

# release
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG -flto")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fpermissive -flto")

set(DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/check
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat
)

set(SRCS
    src/model/aiger.c
    src/model/Settings.cpp
    src/model/CircuitGraph.cpp
    src/model/TernarySim.cpp
    src/model/Model.cpp
    src/model/Log.cpp
    src/check/BackwardChecker.cpp
    src/check/ForwardChecker.cpp
    src/check/BMC.cpp
    src/check/BasicIC3.cpp
    src/model/SATSolver.cpp
    src/model/IncrCheckerHelpers.cpp
    src/main.cpp
    # restart
    src/model/Restart.h
    # minisat
    src/model/MinisatSolver.cpp
    src/sat/minisat/core/Solver.cc
    src/sat/minisat/simp/SimpSolver.cc
    src/sat/minisat/utils/Options.cc
    src/sat/minisat/utils/System.cc
    # minicore
    src/model/MinicoreSolver.cpp
    src/sat/minicore/src/solver.cpp
)

# cadical
if(CADICAL)
    add_definitions(-DCADICAL)
    set(SRCS
        ${SRCS}
        src/model/CadicalSolver.cpp
    )
    set(LIBS ${LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/cadical/build/libcadical.a)
endif()

# kissat
if(KISSAT)
    add_definitions(-DKISSAT)
    set(SRCS
        ${SRCS}
        src/model/KissatSolver.cpp
    )
    set(LIBS ${LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/kissat/build/libkissat.a)
endif()

add_executable(simpleCAR ${SRCS})
target_include_directories(simpleCAR PUBLIC ${DIRS})
target_link_libraries(simpleCAR PUBLIC ${LIBS})

target_link_options(simpleCAR PUBLIC 
    $<$<CONFIG:Release>:-s -static -flto>
)