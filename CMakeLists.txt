cmake_minimum_required(VERSION 4.2.1)
project(MyPackage LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 REQUIRED)

execute_process(
    COMMAND ${Python_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/build_scripts/firmware_to_class.py" 
            "${CMAKE_CURRENT_SOURCE_DIR}/src/firmware/firmware.ino"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/additions.txt"
            "${CMAKE_CURRENT_SOURCE_DIR}/src/board"
            "Board"
    RESULT_VARIABLE SCRIPT_RESULT
)

if(NOT SCRIPT_RESULT EQUAL 0)
  message(FATAL_ERROR "Create a class for the firmware failed!")
endif()

pybind11_add_module(beavs5 src/bindings.cpp)

target_link_libraries(beavs5 PRIVATE 
    boost_coroutine 
    boost_context 
    boost_system
)

install(TARGETS beavs5 DESTINATION .)
