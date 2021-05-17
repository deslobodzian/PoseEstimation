cmake_minimum_required(VERSION 3.1) 
project(pose_estimation)
set(CMAKE_CXX_STANDARD 17)

include_directories(include/)

add_executable(main src/main.cpp)

