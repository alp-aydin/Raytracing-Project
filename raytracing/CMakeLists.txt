cmake_minimum_required(VERSION 3.14)
project(raytracer_min CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)

add_executable(raytracer_min src/main.cpp)
target_link_libraries(raytracer_min PRIVATE ${OpenCV_LIBS})
