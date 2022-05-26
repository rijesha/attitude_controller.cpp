# position_controller/CMakeLists.txt

cmake_minimum_required(VERSION 3.15)

project(position_controller VERSION 0.1.0
        DESCRIPTION "Position Controller library"
        LANGUAGES CXX)

add_library(${PROJECT_NAME} STATIC)
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_sources(${PROJECT_NAME}
    PUBLIC
        position_controller.h
        vector3.h
    PRIVATE
        position_controller.cpp
        vector3.cpp
        fusion-ukf/src/measurementpredictor.cpp
        fusion-ukf/src/statepredictor.cpp
        fusion-ukf/src/stateupdater.cpp
        fusion-ukf/src/datapoint.cpp
        fusion-ukf/src/fusionukf.cpp
        fusion-ukf/src/tools.cpp
        fusion-ukf/headers/
)

target_include_directories(${PROJECT_NAME} PUBLIC ./ fusion-ukf/headers/)
