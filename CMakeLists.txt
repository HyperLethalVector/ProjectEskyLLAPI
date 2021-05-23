cmake_minimum_required(VERSION 3.7.2)
project(libProjectEskyLLAPI)

# Global configs
option(LINK_SHARED_ZED "Link with the ZED SDK shared executable" ON)
option(BUILD_RENDERER_TEST "Build the renderer test program" OFF)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MT")
if(NOT LINK_SHARED_ZED AND MSVC)
    message(FATAL_ERROR "LINK_SHARED_ZED OFF : ZED SDK static libraries not available on Windows")
endif()

# Library finding
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

  find_package (GLM REQUIRED)
  FIND_PATH(DX_INCLUDE_PATH d3d11.h      PATHS "$ENV{DXSDK_DIR}/Include" "C:/Program Files (x86)/Microsoft DirectX SDK/Include" DOC "The directory where D3D11.h resides")
  find_library(D3D11_LIBRARY d3d11.lib   PATHS "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64" DOC "The directory where d3d11.lib resides")
  find_library(D3DX11_LIBRARY d3dx11.lib PATHS "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64" DOC "The directory where d3dx11.lib resides")
  set (D3D11_FOUND "YES")

  find_package(ZED 3)
  find_package(CUDA ${ZED_CUDA_VERSION} EXACT)
endif()

set(D3D11_LIBRARIES ${D3D11_LIB})
mark_as_advanced(D3D11_INCLUDE_PATH D3D11_LIB)
find_library(REALSENSE2_FOUND realsense2 HINTS ${LIBRARY_DIR} REQUIRED)
if(NOT REALSENSE2_FOUND)
  set(REALSENSE2_FOUND "realsense2")
  message(WARN "Failed to find_library(realsense2)")
endif()

# OpenCV
set(OpenCV_STATIC ON)
find_package(OpenCV 4.5.0 REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# includes
include_directories(${ADDITIONAL_INCLUDE_DIRS})
if(WIN32)
  include_directories(${CUDA_INCLUDE_DIRS})
  include_directories(${GLM_INCLUDE_DIRS})
  include_directories(${ZED_INCLUDE_DIRS})
  if(LINK_SHARED_ZED)
    set(ZED_LIBS ${ZED_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_CUDART_LIBRARY})
  else()
    set(ZED_LIBS ${ZED_STATIC_LIBRARIES} ${CUDA_CUDA_LIBRARY} ${CUDA_LIBRARY})
  endif()
endif()
include_directories(./include/)

if(WIN32)  # Windows
  set(CMAKE_EXE_LINKER_FLAGS "-static")
  add_library(libProjectEskyLLAPIIntel SHARED
    ./src/Tracker/IntelT265/main.cpp
    ./src/Tracker/IntelT265/Tracker.cpp
    ./src/Tracker/IntelT265/Serial.cpp
    ./include/DollaryDooFilter.h
    ./include/Serial.h)
  set_target_properties(libProjectEskyLLAPIIntel PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyLLAPIIntel ${DEPENDENCIES} ${REALSENSE2_FOUND}  ${OpenCV_LIBS})
  #ZED
  add_library(libProjectEskyLLAPIZED SHARED
    ./src/Tracker/ZED/main.cpp
    ./src/Tracker/ZED/Tracker_Zed.cpp
    ./src/UnityHooks.def)
  set_source_files_properties(./src/UnityHooks.def PROPERTIES HEADER_FILE_ONLY TRUE)
  set_target_properties(libProjectEskyLLAPIZED PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyLLAPIZED ${DEPENDENCIES} ${ZED_LIBS}  ${D3D11_LIBRARY} ${D3DX11_LIBRARY} ${OpenCV_LIBS} "-static")

  #RGBSensorModule
  add_library(libProjectEskyRGBSensorModule SHARED
    ./src/RGBSensorModule/main.cpp
    ./src/RGBSensorModule/RGBSensorModule.cpp
    ./src/RGBSensorModule/RGBSensorModule.h
    ./src/UnityHooks.def)
  set_source_files_properties(./src/UnityHooks.def PROPERTIES HEADER_FILE_ONLY TRUE)
  set_target_properties(libProjectEskyRGBSensorModule PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyRGBSensorModule ${DEPENDENCIES}  ${OpenCV_LIBS} ${D3D11_LIBRARY} ${D3DX11_LIBRARY} "-static")

  add_executable(ProjectEskyOpenCVCalibration ./src/Calibration/main.cpp)
  target_link_libraries(ProjectEskyOpenCVCalibration ${DEPENDENCIES}  ${OpenCV_LIBS})

  #ARUCOTrackerModule
  add_library(libProjectEskyARUCOTrackerModule SHARED
    ./src/ARUCOTrackerModule/main.cpp
    ./src/ARUCOTrackerModule/ARUCOTrackerModule.cpp
    ./src/ARUCOTrackerModule/ARUCOTrackerModule.h
    ./src/UnityHooks.def)
  set_source_files_properties(./src/UnityHooks.def PROPERTIES HEADER_FILE_ONLY TRUE)
  set_target_properties(libProjectEskyARUCOTrackerModule PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyARUCOTrackerModule ${DEPENDENCIES}  ${OpenCV_LIBS} ${D3D11_LIBRARY} ${D3DX11_LIBRARY} "-static")

  #ARUCOTrackerModule print program
  add_executable(ProjectEskyARUCOTrackerModulePrint
    ./src/ARUCOTrackerModule/main-prog.cpp
    ./src/ARUCOTrackerModule/ARUCOTrackerModule.cpp
    ./src/ARUCOTrackerModule/ARUCOTrackerModule.h
    ./src/UnityHooks.def)
  set_source_files_properties(./src/UnityHooks.def PROPERTIES HEADER_FILE_ONLY TRUE)
  set_target_properties(ProjectEskyARUCOTrackerModulePrint PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(ProjectEskyARUCOTrackerModulePrint ${DEPENDENCIES}  ${OpenCV_LIBS} ${D3D11_LIBRARY} ${D3DX11_LIBRARY} "-static")
else()  # Linux
  add_library(libProjectEskyLLAPIIntel SHARED
    ./src/Tracker/IntelT265/main.cpp
    ./src/Tracker/IntelT265/Tracker.cpp
    ./src/Tracker/IntelT265/LinuxSerialPort.cpp
    ./include/LinuxSerialPort.hpp
    ./include/LinuxSerialException.hpp)
  set_target_properties(libProjectEskyLLAPIIntel PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyLLAPIIntel ${DEPENDENCIES} ${REALSENSE2_FOUND}  ${OpenCV_LIBS})
endif()

# Rendering plugin
add_subdirectory(./src/Renderer/)

if(WIN32)
  message(STATUS "Adding Post build script to copy realsense2.dll to project's binary folder")
  message(STATUS "Will try to copy from ${DLL_DIR} to ${PROJECT_BINARY_OUTPUT_PATH}")
  add_custom_command(
    TARGET libProjectEskyLLAPIIntel POST_BUILD     # Adds a post-build event to rs-getting-started
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
    "${DLL_DIR}/realsense2.dll"                    # <--this is in-file
    ${PROJECT_BINARY_OUTPUT_PATH})                 # <--this is out-file path
endif()
