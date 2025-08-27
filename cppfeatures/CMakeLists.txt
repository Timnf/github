cmake_minimum_required(VERSION 3.10)

project(cppfeatures)

set(CMAKE_CXX_STANDARD 11)

#取当前目录的文件名做执行文件。每个文件做为一个主程序文件
aux_source_directory(. DIR_SRCS)
message(STATUS "DIR_SRCS = ${DIR_SRCS}")

# 取每个文件的命名做执行文件
foreach(sourcefile ${DIR_SRCS})
    get_filename_component(exefile ${sourcefile} NAME_WE)
    add_executable(${exefile} ${sourcefile})
    target_link_libraries(${exefile} pthread)
    message(STATUS "exefile = ${exefile}")
    message(STATUS "sourcefile = ${sourcefile}")
    message(STATUS "DIR_SRCS = ${DIR_SRCS}")
    message(STATUS "CMAKE_CURRENT_SOURCE_DIR = ${CMAKE_CURRENT_SOURCE_DIR}")
    message(STATUS "CMAKE_CURRENT_BINARY_DIR = ${CMAKE_CURRENT_BINARY_DIR}")
    message(STATUS "CMAKE_SOURCE_DIR = ${CMAKE_SOURCE_DIR}")
    message(STATUS "CMAKE_BINARY_DIR = ${CMAKE_BINARY_DIR}")
    message(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
    message(STATUS "PROJECT_BINARY_DIR = ${PROJECT_BINARY_DIR}")
    message(STATUS "CMAKE_CURRENT_LIST_DIR = ${CMAKE_CURRENT_LIST_DIR}")
    message(STATUS "CMAKE_CURRENT_LIST_FILE = ${CMAKE_CURRENT_LIST_FILE}")
    message(STATUS "CMAKE_CURRENT_LIST_LINE = ${CMAKE_CURRENT_LIST_LINE}")
    message(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}")
    message(STATUS "CMAKE_INCLUDE_PATH = ${CMAKE_INCLUDE_PATH}")
    message(STATUS "CMAKE_LIBRARY_PATH = ${CMAKE_LIBRARY_PATH}")
    message(STATUS "CMAKE_SYSTEM_INCLUDE_PATH = ${CMAKE_SYSTEM_INCLUDE_PATH}")
    message(STATUS "CMAKE_SYSTEM_LIBRARY_PATH = ${CMAKE_SYSTEM_LIBRARY_PATH}")
    message(STATUS "CMAKE_SYSTEM_PROCESSOR = ${CMAKE_SYSTEM_PROCESSOR}")
    message(STATUS "CMAKE_SYSTEM_NAME = ${CMAKE_SYSTEM_NAME}")
    message(STATUS "CMAKE_SYSTEM_VERSION = ${CMAKE_SYSTEM_VERSION}")
endforeach()





