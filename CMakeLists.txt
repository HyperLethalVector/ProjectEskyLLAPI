cmake_minimum_required(VERSION 3.7.2)

# Proeject name - should not contain whitespace 
project(libProjectEskyLLAPI)

#========== Global Configurations =============#
#----------------------------------------------#
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)

if (NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()
# Set the C++ standard for all targets (It sets the flags
# (-std=c++11, -std=c++14 ...) on Clang or GCC. and /std=c++17 on MSVC
# OPTIONAL:
#---------------------------------
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()
set(CMAKE_VERBOSE_MAKEFILE ON)

if(WIN32)
    if("${CMAKE_SIZEOF_VOID_P}" STREQUAL "8")
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x64")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x64")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    else()
        set(LIBRARY_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\lib\\x86")          # TODO: Update this variable to correct path - folder where realsense2.lib is found
        set(DLL_DIR "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\bin\\x86")              # TODO: Update this variable to correct path - folder where realsense2.dll is found
    endif()
    set(PROJECT_BINARY_OUTPUT_PATH "${PROJECT_BINARY_DIR}/\$\(Configuration\)")                # TODO: Update this variable to correct path - folder to which your project will compile
    set(ADDITIONAL_INCLUDE_DIRS "C:\\Program Files (x86)\\Intel RealSense SDK 2.0\\include")   # TODO: Update this variable to correct path - folder where librealsense2 folder is found
endif()

#find_package(ZED 3 REQUIRED)
#find_package(CUDA ${ZED_CUDA_VERSION} EXACT REQUIRED)

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()



find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${ADDITIONAL_INCLUDE_DIRS})

#include_directories(${CUDA_INCLUDE_DIRS})
#include_directories(${ZED_INCLUDE_DIRS})
include_directories(${GLEW_INCLUDE_DIR})
include_directories(./include)
include_directories(${GLUT_INCLUDE_DIR})
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
#========== Targets Configurations ============#
#    if (LINK_SHARED_ZED)
#        SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
#    else()
#        SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
#    endif()
# Build an executable (Unix-like OSes generates ./ProjectCupboardLLAPI, on
# Windows ./ProjectCupboardLLAPI.exe)
# ..........................................
add_library(libProjectEskyLLAPI SHARED ./src/mainLibTrackingIncluded.cpp ./src/OpenGLControl.cpp ./src/Tracker.cpp ./src/RenderScene.cpp ./src/Shaders.cpp ./src/Win_OpenGLApp.cpp)
add_library(libProjectEskyLLAPIRendererStandalone SHARED ./src/mainLibRendererStandalone.cpp ./src/OpenGLControl.cpp ./src/RenderScene.cpp ./src/Shaders.cpp ./src/Win_OpenGLApp.cpp)
set_target_properties(libProjectEskyLLAPI PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
# Set executable dependency libraries
# Equivalent to pass flags -lGL, -lGLU and -lglut 
target_link_libraries(libProjectEskyLLAPI ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${DEPENDENCIES} ${REALSENSE2_FOUND})
target_link_libraries(libProjectEskyLLAPIRendererStandalone ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES} ${DEPENDENCIES})
if(WIN32)
message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
add_custom_command(TARGET libProjectEskyLLAPI POST_BUILD        # Adds a post-build event to rs-getting-started
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${DLL_DIR}/realsense2.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
endif()
