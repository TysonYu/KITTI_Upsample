cmake_minimum_required(VERSION 3.5)
project(upsampling)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -g -O2 -ffast-math")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O2 -ffast-math")

find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.1 REQUIRED )

include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        "/usr/include/eigen3"
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
        ${PROJECT_SOURCE_DIR}/src/data_loader.cpp
        ${PROJECT_SOURCE_DIR}/src/mrf.cpp
)

add_executable(main main.cpp)

target_link_libraries(main
        ${PROJECT_NAME}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)
