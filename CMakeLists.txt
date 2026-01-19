cmake_minimum_required(VERSION 3.30)
project(MyPackage LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

# TODO: Figure out a nicer way to do this command
set(OUTPUT_NAME "${CMAKE_CURRENT_BINARY_DIR}/src/board")
set(OUTPUT_FILES "${CMAKE_CURRENT_BINARY_DIR}/src/board.h"
                 "${CMAKE_CURRENT_BINARY_DIR}/src/board.cpp")
set(FILE_ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/firmware_to_class.py"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/firmware/firmware.ino"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/additions.txt")

add_custom_command(
    OUTPUT ${OUTPUT_FILES}
    COMMAND ${Python_EXECUTABLE}
            ${FILE_ARGS}
            ${OUTPUT_NAME}
            "Board"
    DEPENDS ${FILE_ARGS}
    COMMENT "Generating firmware class"
    VERBATIM
)

file(GLOB_RECURSE SOURCE_FILES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

find_package(Boost REQUIRED COMPONENTS system)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 REQUIRED)

pybind11_add_module(beavs_sim ${SOURCE_FILES} ${OUTPUT_FILES})

target_include_directories(beavs_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/src"
)

target_link_libraries(beavs_sim PRIVATE
    boost_coroutine
    boost_context
    boost_system
    atomic
)

install(TARGETS beavs_sim DESTINATION .)
