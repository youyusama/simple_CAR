cmake_minimum_required(VERSION 3.10)

# Compiler
if(NOT DEFINED CMAKE_C_COMPILER AND NOT DEFINED CMAKE_CXX_COMPILER)
    set(CMAKE_C_COMPILER   clang   CACHE STRING "C compiler" FORCE)
    set(CMAKE_CXX_COMPILER clang++ CACHE STRING "CXX compiler" FORCE)
endif()

project(simpleCAR VERSION 0.1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(ENABLE_SANITIZERS "Enable Address/UB Sanitizers and extra debug flags" OFF)

# Debug
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

# Release
set(CMAKE_C_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -fpermissive")

# Enable IPO/LTO only for Release
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON)

# LLD linker and ThinLTO
option(USE_LLD "Use lld linker (recommended with Clang)" ON)
option(ENABLE_THINLTO "Use ThinLTO in Release (recommended)" ON)

if(USE_LLD)
  add_link_options($<$<COMPILE_LANG_AND_ID:C,Clang>:-fuse-ld=lld>
                   $<$<COMPILE_LANG_AND_ID:CXX,Clang>:-fuse-ld=lld>)
endif()

if(ENABLE_THINLTO)
  add_compile_options($<$<AND:$<CONFIG:Release>,$<COMPILE_LANG_AND_ID:C,Clang>>:-flto=thin>
                      $<$<AND:$<CONFIG:Release>,$<COMPILE_LANG_AND_ID:CXX,Clang>>:-flto=thin>)
  add_link_options($<$<AND:$<CONFIG:Release>,$<LINK_LANG_AND_ID:C,Clang>>:-flto=thin>
                   $<$<AND:$<CONFIG:Release>,$<LINK_LANG_AND_ID:CXX,Clang>>:-flto=thin>)
endif()

if(ENABLE_THINLTO AND USE_LLD)
  add_link_options($<$<AND:$<CONFIG:Release>,$<LINK_LANG_AND_ID:CXX,Clang>>:-Wl,--thinlto-jobs=all>)
endif()


set(DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model
    ${CMAKE_CURRENT_SOURCE_DIR}/src/alg
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat
)

set(SRCS
    src/model/aiger.c
    src/model/CircuitGraph.cpp
    src/model/TernarySim.cpp
    src/model/Model.cpp
    src/model/DAGCNFSimplifier.cpp
    src/Settings.cpp
    src/Log.cpp
    src/alg/BCAR.cpp
    src/alg/FCAR.cpp
    src/alg/BMC.cpp
    src/alg/BasicIC3.cpp
    src/sat/SATSolver.cpp
    src/IncrCheckerHelpers.cpp
    src/SimpleCAR.cpp
    src/main.cpp
    # minisat
    src/sat/MinisatSolver.cpp
    src/sat/minisat/core/Solver.cc
    src/sat/minisat/simp/SimpSolver.cc
    src/sat/minisat/utils/Options.cc
    src/sat/minisat/utils/System.cc
    # minicore
    src/sat/MinicoreSolver.cpp
    src/sat/minicore/src/solver.cpp
)

# cadical
if(CADICAL)
    add_definitions(-DCADICAL)
    set(SRCS
        ${SRCS}
        src/sat/CadicalSolver.cpp
    )
    set(LIBS ${LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/cadical/build/libcadical.a)
endif()

# kissat
if(KISSAT)
    add_definitions(-DKISSAT)
    set(SRCS
        ${SRCS}
        src/sat/KissatSolver.cpp
    )
    set(LIBS ${LIBS} ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/kissat/build/libkissat.a)
endif()

add_executable(simpleCAR ${SRCS})
target_include_directories(simpleCAR PUBLIC ${DIRS})
target_link_libraries(simpleCAR PUBLIC ${LIBS})

add_executable(minicore_bench
    src/sat/minicore/src/bench.cpp
    src/sat/minicore/src/solver.cpp
)
target_include_directories(minicore_bench PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/minicore/src)

if(APPLE)
    # On macOS, avoid -static (causes missing crt0.o); keep strip and LTO.
    target_link_options(simpleCAR PUBLIC $<$<CONFIG:Release>:-s>)
else()
    # On other Unices we may use static linking for release builds.
    target_link_options(simpleCAR PUBLIC $<$<CONFIG:Release>:-s -static>)
endif()

message(STATUS "C compiler:   ${CMAKE_C_COMPILER}")
message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")
