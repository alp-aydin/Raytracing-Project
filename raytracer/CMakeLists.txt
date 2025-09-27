cmake_minimum_required(VERSION 3.21)
project(raytracer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Find packages
find_package(OpenCV REQUIRED COMPONENTS imgcodecs core)
find_package(nlohmann_json CONFIG REQUIRED)

# Fetch Catch2
include(FetchContent)
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# Create a library for raytracer code (excluding main.cpp)
file(GLOB RAYTRACER_SRC CONFIGURE_DEPENDS 
     "${CMAKE_SOURCE_DIR}/src/*.cpp")
list(REMOVE_ITEM RAYTRACER_SRC "${CMAKE_SOURCE_DIR}/src/main.cpp")

add_library(raytracer_lib ${RAYTRACER_SRC})
target_include_directories(raytracer_lib PUBLIC ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(raytracer_lib 
    PUBLIC nlohmann_json::nlohmann_json ${OpenCV_LIBS})

# Main executable
add_executable(ray src/main.cpp)
target_link_libraries(ray PRIVATE raytracer_lib)

# Test executable
file(GLOB TEST_SRC CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/tests/*.cpp")
add_executable(raytracer_tests ${TEST_SRC})
target_link_libraries(raytracer_tests 
    PRIVATE raytracer_lib Catch2::Catch2WithMain)

# Discover tests
list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(raytracer_tests)

# Compiler flags
if (MSVC)
  target_compile_options(raytracer_lib PRIVATE /W4 /permissive- /MP)
else()
  target_compile_options(raytracer_lib PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()   