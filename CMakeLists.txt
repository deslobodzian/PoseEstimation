cmake_minimum_required(VERSION 3.1) 
project(pose_estimation)

option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
SET(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package(ZED 3 REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

IF(NOT WIN32)
     SET(SPECIAL_OS_LIBS "pthread" "X11")
    add_definitions(-Wno-write-strings)
ENDIF()

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${ZED_LIBRARY_DIRS})
link_directories(${GLEW_LIBRARY_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})

FILE(GLOB_RECURSE SRC_FILES src/*.cpp)
FILE(GLOB_RECURSE HDR_FILES include/*.hpp)

add_executable(${PROJECT_NAME} ${HDR_FILES} ${SRC_FILES})
add_definitions(-O3)

if (LINK_SHARED_ZED)
	SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
else()
    SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
endif()

TARGET_LINK_LIBRARIES(${PROJECT_NAME} 
                        ${SPECIAL_OS_LIBS} 
                        ${ZED_LIBS} 
                        ${OPENGL_LIBRARIES}
                        ${GLUT_LIBRARY}
                        ${GLEW_LIBRARIES})
