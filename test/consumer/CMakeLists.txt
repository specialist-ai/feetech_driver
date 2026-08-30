# A downstream project, built against the INSTALLED package rather than the
# build tree. This is what catches an export that names a dependency the
# consumer is never told to find.
cmake_minimum_required(VERSION 3.16)
project(feetech_driver_consumer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(feetech_driver REQUIRED)

add_executable(consumer main.cpp)
target_link_libraries(consumer PRIVATE feetech::feetech)
