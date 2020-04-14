# attitude_controller/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(attitude_controller VERSION 0.1.0
        DESCRIPTION "Attitude Controller library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        attitude_controller.h
        vector3.h
    PRIVATE
        attitude_controller.cpp
        vector3.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC ./)
