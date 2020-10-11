cmake_minimum_required(VERSION 3.16)
project(dlmalloc_src)

set(CMAKE_CXX_STANDARD 14)

add_executable(dlmalloc_src imalloc.cpp main.cpp)