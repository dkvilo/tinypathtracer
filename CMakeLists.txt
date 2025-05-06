cmake_minimum_required(VERSION 3.10)
project(pathtracer)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

add_executable(pathtracer Src/Main.cpp Src/Math.cpp Src/Camera.cpp Src/Mesh.cpp)
target_link_libraries(pathtracer SDL2)