cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu11")

project(charging-station-comm C)

find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML REQUIRED libcyaml)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -DMAVLINK_USE_MESSAGE_INFO -D_STDDEF_H -D__need_size_t -D__need_NULL")

add_executable(charging-station-comm main.c endpoint.c config.c)

target_include_directories(charging-station-comm PUBLIC ${YAML_INCLUDE_DIRS})

target_link_libraries(charging-station-comm ${YAML_LIBRARIES})


