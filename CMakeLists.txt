cmake_minimum_required(VERSION 3.1.0)

set(CMAKE_C_FLAGS "-std=gnu11")

project(mavlink-fast-switch C)

set(VERSION_MAJOR 0)
set(VERSION_MINOR 1)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DVERSION_MAJOR=${VERSION_MAJOR} -DVERSION_MINOR=${VERSION_MINOR}")

find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML REQUIRED libcyaml)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -DMAVLINK_USE_MESSAGE_INFO -D_STDDEF_H -D__need_size_t -D__need_NULL")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")
set(CMAKE_LINKER_FLAGS_DEBUG "${CMAKE_LINKER_FLAGS_DEBUG} -fno-omit-frame-pointer -fsanitize=address,undefined,leak")

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

if(NOT EXISTS "${PROJECT_SOURCE_DIR}/libs/mavlink/v2.0")
    message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
endif()

if(NOT DEFINED MAVLINK_DIALECT)
    set(MAVLINK_DIALECT "ardupilotmega")
    message(WARNING "MAVLink dialect is not set. Using ardupilotmega by default!")
endif()

configure_file(
  "${PROJECT_SOURCE_DIR}/mavlink_dialect.h.in"
  "${PROJECT_BINARY_DIR}/mavlink_dialect.h"
)

add_executable(mavlink-fast-switch main.c endpoint.c config.c)
target_include_directories(mavlink-fast-switch PUBLIC ${YAML_INCLUDE_DIRS} ${PROJECT_BINARY_DIR})
target_link_libraries(mavlink-fast-switch ${YAML_LIBRARIES})

configure_file("${PROJECT_SOURCE_DIR}/mavlink-fast-switch@.service.cmake"
    "${PROJECT_BINARY_DIR}/mavlink-fast-switch@.service")

install(TARGETS mavlink-fast-switch DESTINATION bin)
install(FILES "${PROJECT_BINARY_DIR}/mavlink-fast-switch@.service" DESTINATION 
    "/lib/systemd/system")
install(FILES "${PROJECT_SOURCE_DIR}/config.yaml" DESTINATION 
    "/etc/mavlink-fast-switch/" RENAME "example.yaml")
