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
FIND_PATH(DX_INCLUDE_PATH d3d11.h     PATHS     "$ENV{DXSDK_DIR}/Include" "C:/Program Files (x86)/Microsoft DirectX SDK/Include"     DOC "The directory where D3D11.h resides")  
find_library(D3D11_LIBRARY d3d11.lib     PATHS     "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64"     DOC "The directory where d3d11.lib resides")  
find_library(D3DX11_LIBRARY d3dx11.lib     PATHS     "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64"     DOC "The directory where d3dx11.lib resides") 
set (D3D11_FOUND "YES") 
set (D3D11_LIBRARIES ${D3D11_LIB}) 
mark_as_advanced (D3D11_INCLUDE_PATH D3D11_LIB)  

find_package(ZED 3)
find_package(CUDA ${ZED_CUDA_VERSION} EXACT)

find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
    SET(REALSENSE2_FOUND "realsense2")
    message(WARN "Failed to find_library(realsense2)")
endif()


include_directories(${ADDITIONAL_INCLUDE_DIRS})

include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${ZED_INCLUDE_DIRS})
include_directories(./include)
#========== Targets Configurations ============#
    if (LINK_SHARED_ZED)
        SET(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
    else()
        SET(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY}) 
    endif()

add_library(libProjectEskyLLAPIIntel SHARED ./src/mainLibTrackingIncluded.cpp ./src/Tracker.cpp)
add_library(libProjectEskyLLAPIZED SHARED ./src/mainLibTrackingIncludedZED.cpp ./src/Tracker_Zed.cpp ./src/UnityHooks.def)
set_source_files_properties(./src/UnityHooks.def PROPERTIES HEADER_FILE_ONLY TRUE)
set_target_properties(libProjectEskyLLAPIIntel PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
set_target_properties(libProjectEskyLLAPIZED PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
target_link_libraries(libProjectEskyLLAPIIntel ${DEPENDENCIES} ${REALSENSE2_FOUND})
target_link_libraries(libProjectEskyLLAPIZED ${DEPENDENCIES} ${ZED_LIBS}  ${D3D11_LIBRARY} ${D3DX11_LIBRARY})
if(WIN32)
message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
add_custom_command(TARGET libProjectEskyLLAPIIntel POST_BUILD        # Adds a post-build event to rs-getting-started
    COMMAND ${CMAKE_COMMAND} -E copy_if_different              # which executes "cmake - E copy_if_different..."
        "${DLL_DIR}/realsense2.dll"                            # <--this is in-file
        ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
endif()
