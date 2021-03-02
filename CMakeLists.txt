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

set(OpenGL_GL_PREFERENCE GLVND)
set(FREEGLUT_DIR "/")
set(EIGEN3_INCLUDE_DIR "/usr/include/eigen3/")
find_package(realsense2 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(include
    ${realsense_INCLUDE_DIR}   
    ${GLUT_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(realsense-handler src/realsense_handler.cpp src/realsense_handler.h)
target_link_libraries(realsense-handler
    ${realsense2_LIBRARY}
)

add_library(shader src/shader.cpp src/shader.h)
target_link_libraries(shader
    ${GLUT_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
)

# Build my hello-world application for probing read framerate
add_executable(rs-hello-realsense src/rs-hello-realsense.cpp)
target_link_libraries(rs-hello-realsense
    realsense-handler
    shader
    ${realsense2_LIBRARY} 
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::GLEW
    Eigen3::Eigen
)

install(TARGETS rs-hello-realsense RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
