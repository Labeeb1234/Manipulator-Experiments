cmake_minimum_required(VERSION 3.5)
project(uarm_cpp_ws)

set(CMAKE_CXX_STANDARD 11)

# Find the uArm SDK package
include_directories(${PROJECT_SOURCE_DIR}/uArm-SDK/include)
link_directories(${PROJECT_SOURCE_DIR}/uArm-SDK/lib)

add_executable(uarm_ports src/uArmtester.cpp)

# Link against the uArm library
target_link_libraries(uarm_ports uarm)
