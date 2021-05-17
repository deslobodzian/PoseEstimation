cmake_minimum_required(VERSION 3.1) 
project(pose_estimation)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(ZED 3 REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

include_directories(include/)
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIRS})

add_executable(${PROJECT_NAME} src/main.cpp)
add_definitions(-O3)

if (LINK_SHARED_ZED)
	SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()


target_link_libraries(${PROJECT_NAME} ${ZED_LIBS})

