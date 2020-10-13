cmake_minimum_required(VERSION 3.10)
project(iMalloc)

set(CMAKE_CXX_FLAGS "-Wno-error=deprecated-declarations -Wno-deprecated-declarations")

set(CMAKE_CXX_STANDARD 14)

add_executable(iMalloc iMalloc.cpp main.cpp)