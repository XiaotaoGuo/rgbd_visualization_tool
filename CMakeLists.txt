cmake_minimum_required(VERSION 2.8)
project(rgbd_visualization_tool)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV 2.4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS})

add_executable(3dreconstruct rgbd_visualize.cpp)
target_link_libraries(3dreconstruct
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES})
