cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(pcd_tool)
find_package(PCL 1.3 REQUIRED COMPONENTS)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pcd_processor 
    include/pcd_processor.cpp
    include/pcd_processor.h
    )
target_link_libraries(pcd_processor
    ${PCL_LIBRARIES}
)

add_executable(pcd_processor_tool src/main.cpp)
target_link_libraries(pcd_processor_tool pcd_processor)