cmake_minimum_required(VERSION 3.10)

project(simpleCAR VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# debug
set(CMAKE_C_FLAGS_DEBUG "-g -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-g -ggdb -fpermissive")

# release
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fpermissive")

set(DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/check
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat
)

set(SRCS
    src/model/aiger.c
    src/model/Settings.cpp
    src/model/Model.cpp
    src/model/Log.cpp
    src/check/BackwardChecker.cpp
    src/check/ForwardChecker.cpp
    src/check/BMC.cpp
    src/model/SATSolver.cpp
    src/model/IncrCheckerHelpers.cpp
    src/main.cpp
    # minisat
    src/model/MinisatSolver.cpp
    src/sat/minisat/core/Solver.cc
    src/sat/minisat/simp/SimpSolver.cc
    src/sat/minisat/utils/Options.cc
    src/sat/minisat/utils/System.cc
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

add_executable(simpleCAR ${SRCS})
target_include_directories(simpleCAR PUBLIC ${DIRS})
target_link_libraries(simpleCAR PUBLIC ${LIBS})
target_link_options(simpleCAR PUBLIC -s -static -flto)