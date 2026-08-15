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
set(CMAKE_C_FLAGS_DEBUG "-g -O0 -gdwarf-4 -ggdb")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 -gdwarf-4 -ggdb -fpermissive")

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
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model/aiger
    ${CMAKE_CURRENT_SOURCE_DIR}/src/alg
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat
    ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/minisat
    ${CMAKE_CURRENT_SOURCE_DIR}/src/frontend/btor2
    ${CMAKE_CURRENT_SOURCE_DIR}/src/wordlevel
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/btor2tools/src
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/btor2tools/deps/install/include
)

set(MINICORE_LIB_SRCS
    src/sat/minicore/src/solver.cpp
)

set(MINICORE_CLI_SRCS
    src/sat/minicore/src/main.cpp
    src/sat/minicore/src/dimacs.cpp
)

set(SRCS
    src/model/aiger/aiger.c
    src/model/CircuitGraph.cpp
    src/model/TernarySim.cpp
    src/model/SATSim.cpp
    src/model/Model.cpp
    src/model/DAGCNFSimplifier.cpp
    src/frontend/btor2/Btor2Frontend.cpp
    src/wordlevel/model/WLArrayAbstraction.cpp
    src/wordlevel/WLBitVector.cpp
    src/wordlevel/model/WLModel.cpp
    src/wordlevel/model/WLPackageResize.cpp
    src/wordlevel/WLSimulator.cpp
    src/wordlevel/WLChecker.cpp
    src/wordlevel/WLCegar.cpp
    src/wordlevel/WLMemoryBMC.cpp
    src/wordlevel/model/WLBitblastor.cpp
    deps/btor2tools/src/btor2parser/btor2parser.c
    deps/btor2tools/src/btorsim/btorsimbv.c
    deps/btor2tools/src/btorsim/btorsimrng.c
    src/Settings.cpp
    src/Log.cpp
    src/WitnessBuilder.cpp
    src/alg/BCAR.cpp
    src/alg/FCAR.cpp
    src/alg/BMC.cpp
    src/alg/KIND.cpp
    src/alg/IC3.cpp
    src/alg/L2S.cpp
    src/alg/KFAIR.cpp
    src/alg/RLive.cpp
    src/sat/SATSolver.cpp
    src/IncrCheckerHelpers.cpp
    src/SimpleCAR.cpp
    src/main.cpp
    # minisat
    src/sat/MinisatSolver.cpp
    src/sat/minisat/minisat/core/Solver.cc
    src/sat/minisat/minisat/simp/SimpSolver.cc
    src/sat/minisat/minisat/utils/Options.cc
    src/sat/minisat/minisat/utils/System.cc
    # minicore
    src/sat/MinicoreSolver.cpp
)

# cadical
set(CADICAL_LIB ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/cadical/build/libcadical.a)
if(NOT EXISTS ${CADICAL_LIB})
    message(FATAL_ERROR
        "CaDiCaL is required but '${CADICAL_LIB}' was not found. "
        "Build it first under src/sat/cadical/ (for example: ./configure --competition && make).")
endif()

set(SRCS
    ${SRCS}
    src/sat/CadicalSolver.cpp
)
set(LIBS ${LIBS} ${CADICAL_LIB})

set(BOOLECTOR_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/btor2tools/deps/install/lib/libboolector.a)
set(BOOLECTOR_BTOR2PARSER_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/btor2tools/deps/boolector/deps/install/lib/libbtor2parser.a)
set(BOOLECTOR_LINGELING_LIB
    ${CMAKE_CURRENT_SOURCE_DIR}/deps/btor2tools/deps/boolector/deps/install/lib/liblgl.a)
if(NOT EXISTS ${BOOLECTOR_LIB})
    message(FATAL_ERROR "Native BTOR2 frontend requires '${BOOLECTOR_LIB}'")
endif()
set(LIBS ${LIBS}
    ${BOOLECTOR_LIB}
    ${BOOLECTOR_BTOR2PARSER_LIB}
    ${BOOLECTOR_LINGELING_LIB}
    pthread)

# Kissat is required by the native word-level memory BMC.
set(KISSAT_LIB ${CMAKE_CURRENT_SOURCE_DIR}/src/sat/kissat/build/libkissat.a)
if(NOT EXISTS ${KISSAT_LIB})
    message(FATAL_ERROR "WL memory BMC requires '${KISSAT_LIB}'")
endif()
add_definitions(-DKISSAT)
set(SRCS ${SRCS} src/sat/KissatSolver.cpp)
set(LIBS ${LIBS} ${KISSAT_LIB})

add_executable(simpleCAR ${SRCS})
target_include_directories(simpleCAR PUBLIC ${DIRS})
add_library(minicore_lib STATIC ${MINICORE_LIB_SRCS})
target_include_directories(minicore_lib PUBLIC ${DIRS})

add_executable(minicore ${MINICORE_CLI_SRCS})
target_include_directories(minicore PUBLIC ${DIRS})
target_link_libraries(minicore PUBLIC minicore_lib)

target_link_libraries(simpleCAR PUBLIC ${LIBS} minicore_lib)

if(APPLE)
    # On macOS, avoid -static (causes missing crt0.o); keep strip and LTO.
    target_link_options(simpleCAR PUBLIC $<$<CONFIG:Release>:-s>)
else()
    # On other Unices we may use static linking for release builds.
    target_link_options(simpleCAR PUBLIC $<$<CONFIG:Release>:-s -static>)
endif()

message(STATUS "C compiler:   ${CMAKE_C_COMPILER}")
message(STATUS "CXX compiler: ${CMAKE_CXX_COMPILER}")
