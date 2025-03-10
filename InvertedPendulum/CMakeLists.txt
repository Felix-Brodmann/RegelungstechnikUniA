cmake_minimum_required(VERSION 3.10)
project(InvertedPendulum)

set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES 
    src/main.cpp
    src/mpu6050.cpp
    src/complementaryFilter.cpp
    src/ev3Motor.cpp
)

add_executable(InvertedPendulum ${SOURCE_FILES})
    