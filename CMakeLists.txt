cmake_minimum_required(VERSION 3.20)
get_filename_component(PROJECT_NAME ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT_NAME} LANGUAGES CXX)

include(CTest)

enable_testing() 

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/src
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -std=c++23 -g3 -Wall -Wextra -DDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_DEBUG "")

set(CMAKE_CXX_COMPILER_LAUNCHER ccache)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -std=c++23 -march=native -DNDEBUG")
set(CMAKE_EXE_LINKER_FLAGS_RELEASE "-flto")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_executable(${PROJECT_NAME} src/main.cpp)