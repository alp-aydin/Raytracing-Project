cmake_minimum_required(VERSION 3.21)
project(raytracer CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS imgcodecs core)
find_package(nlohmann_json CONFIG REQUIRED)

# grab everything from src/
file(GLOB SRC CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/*.cpp")

add_executable(ray ${SRC})

# headers live in src/ too
target_include_directories(ray PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(ray
  PRIVATE nlohmann_json::nlohmann_json
          ${OpenCV_LIBS}
)

if (MSVC)
  target_compile_options(ray PRIVATE /W4 /permissive- /MP)
else()
  target_compile_options(ray PRIVATE -Wall -Wextra -Wpedantic -O2)
endif()
