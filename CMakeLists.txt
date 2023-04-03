cmake_minimum_required(VERSION 3.20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(opengdemu)

# Check to find docker
find_program(Docker_EXECUTABLE docker)
if(NOT Docker_EXECUTABLE)
    message(FATAL_ERROR "Cannot find the docker executable!")
endif()


add_subdirectory(CPU)
add_subdirectory(FPGA)
