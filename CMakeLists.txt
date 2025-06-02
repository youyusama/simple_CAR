cmake_minimum_required(VERSION 3.10)

project(simpleCAR VERSION 0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_C_FLAGS "-g")
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")

set(CMAKE_CXX_FLAGS "-g -fpermissive")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

set(LINK_FLAGS "-s -g -lz -lpthread -static -flto")

set(CMAKE_SHARED_LINKER_FLAGS "${LINK_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS "${LINK_FLAGS}")

set(SRCS
    src/model/aiger.c
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
if(CADICAL)
set(SRCS
    ${SRCS}
    src/model/CadicalSolver.cpp
)
endif()

add_executable(simpleCAR ${SRCS}) 

target_include_directories(simpleCAR
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/check
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat
)


if(CADICAL)
add_definitions(-DCADICAL)
target_link_libraries(simpleCAR
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/cadical/build/libcadical.a
)
endif()