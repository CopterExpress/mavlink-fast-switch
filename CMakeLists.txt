cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu99")

project(charging-station-comm C)

set(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -Wall")

add_executable(charging-station-comm main.c)
