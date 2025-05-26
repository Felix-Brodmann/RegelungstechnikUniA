cmake_minimum_required(VERSION 3.10)
project(InvertedPendulum)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

set(SOURCE_FILES 
    src/main.cpp
)

add_executable(InvertedPendulum ${SOURCE_FILES})

target_include_directories(my_executable PRIVATE /usr/include)
target_include_directories(my_executable PRIVATE /usr/local/include)
target_include_directories(my_executable PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(InvertedPendulum
    pthread
    pigpio
)
