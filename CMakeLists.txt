cmake_minimum_required(VERSION 3.1) 
project(pose_estimation)
set(CMAKE_CXX_STANDARD 17)

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(include/)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)
add_definitions(-std=c++17 -O3)

target_link_libraries(${PROJECT_NAME} ${ZED_LIBS})

