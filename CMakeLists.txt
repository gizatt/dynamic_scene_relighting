cmake_minimum_required(VERSION 3.1.0)

project(dynamic_scene_relighting)

# Make project require C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(realsense2 REQUIRED)

# Build my hello-world application for probing read framerate
add_executable(rs-hello-realsense src/rs-hello-realsense.cpp)
include_directories(include ${realsense_INCLUDE_DIR})
target_link_libraries(rs-hello-realsense ${realsense2_LIBRARY})

install(TARGETS rs-hello-realsense RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
