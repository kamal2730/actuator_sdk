cmake_minimum_required(VERSION 3.10)
project(motion_sdk LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(motion_sdk
    src/usb_serial.cpp
    src/servo.cpp
)

target_include_directories(motion_sdk PUBLIC include)

add_executable(test_servo examples/test_servo.cpp)
target_link_libraries(test_servo motion_sdk)
