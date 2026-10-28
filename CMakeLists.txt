# For IDEs (CLion etc.); on the command line ./build.sh does the same.
cmake_minimum_required(VERSION 3.16)
project(rcpsp_petri CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)   # the search is much slower without optimisation
endif()

add_executable(rcpsp src/main.cpp)   # one translation unit: main.cpp includes every header
target_include_directories(rcpsp PRIVATE HOG2/gui/STUB)
if(WIN32)
    target_link_libraries(rcpsp psapi)   # peak-memory column
endif()
