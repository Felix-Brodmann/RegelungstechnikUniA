cmake_minimum_required(VERSION 3.10)
project(InvertedPendulum)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

set(SOURCE_FILES 
    src/main.cpp
    src/mpu6050.cpp
    src/lsm6ds3.cpp
    src/complementaryFilter.cpp
    src/ev3Motor.cpp
)

add_executable(InvertedPendulum ${SOURCE_FILES})

target_link_libraries(InvertedPendulum
    sfml-graphics
    sfml-window
    sfml-system
    ${GPIOD_LIBRARIES})