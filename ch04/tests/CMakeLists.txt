
cmake_minimum_required(VERSION 3.10)
project(myapp)

set(CMAKE_CXX_STANDARD 14)

add_executable(myapp myapp.cpp src/rttest.cpp)

target_include_directories(myapp PRIVATE include)