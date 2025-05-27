cmake_minimum_required(VERSION 3.10)
project(InvertedPendulum)

set(CMAKE_CXX_STANDARD 17)

find_package(Threads REQUIRED)

set(SOURCE_FILES 
    src/main.cpp
)

add_executable(InvertedPendulum ${SOURCE_FILES})

target_include_directories(InvertedPendulum PRIVATE /usr/include)
target_include_directories(InvertedPendulum PRIVATE /usr/local/include)
target_include_directories(InvertedPendulum PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(InvertedPendulum
    pthread
    pigpio
)
