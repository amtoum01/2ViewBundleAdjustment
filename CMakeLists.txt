cmake_minimum_required(VERSION 3.0)

project(SFM)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(my_executable src/main.cpp include/control_points.cpp include/correspondences.cpp include/ls_solver.cpp include/preprocess.cpp)

target_link_libraries(my_executable ${OpenCV_LIBS})