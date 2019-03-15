cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu11")

project(mavlink-fast-switch C)

find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML REQUIRED libcyaml)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -DMAVLINK_USE_MESSAGE_INFO -D_STDDEF_H -D__need_size_t -D__need_NULL")

set (CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")
set (CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")

add_executable(mavlink-fast-switch main.c endpoint.c config.c)

target_include_directories(mavlink-fast-switch PUBLIC ${YAML_INCLUDE_DIRS})

target_link_libraries(mavlink-fast-switch ${YAML_LIBRARIES})


