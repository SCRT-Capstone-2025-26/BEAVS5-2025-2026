cmake_minimum_required(VERSION 3.21)
project(beavs_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)

include(FetchContent)
include(CheckLinkerFlag)

# TODO: Figure out a nicer way to do this command
set(OUTPUT_NAME "${CMAKE_CURRENT_BINARY_DIR}/generated/board")
set(OUTPUT_FILES "${CMAKE_CURRENT_BINARY_DIR}/generated/board.h"
                 "${CMAKE_CURRENT_BINARY_DIR}/generated/board.cpp")
set(FILE_ARGS "${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/firmware_to_class.py"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/firmware/firmware.ino"
              "${CMAKE_CURRENT_SOURCE_DIR}/src/additions.txt")

file(GLOB SOURCE_FILES src/*.cpp)

find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)

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

FetchContent_Declare(
    boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.90.0/boost-1.90.0-cmake.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)

set(BOOST_ENABLE_CMAKE ON)
set(BOOST_INCLUDE_LIBRARIES system context coroutine2)

FetchContent_MakeAvailable(boost)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v3.0.1
)

FetchContent_MakeAvailable(pybind11)

pybind11_add_module(beavs_sim ${SOURCE_FILES} ${OUTPUT_FILES})

target_include_directories(beavs_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
    "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

target_link_libraries(beavs_sim PRIVATE
    Boost::system
    Boost::context
    Boost::coroutine2
)

check_linker_flag(CXX "-latomic" CXX_LINKS_ATOMIC)

if(CXX_LINKS_ATOMIC)
    target_link_libraries(beavs_sim PRIVATE atomic)
endif()

if(MSVC)
  target_compile_options(beavs_sim PRIVATE /W4 /WX)
else()
  target_compile_options(beavs_sim PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

install(TARGETS beavs_sim DESTINATION .)
