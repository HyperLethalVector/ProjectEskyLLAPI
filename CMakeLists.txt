cmake_minimum_required(VERSION 3.7.2)
project(libProjectEskyLLAPI)

# Global configs 
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MT")

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


find_package (GLM REQUIRED)
find_package( xvsdk REQUIRED )
set(xvsdk_INCLUDE ${xvsdk_INCLUDE_DIRS}/xvsdk})
include_directories( ${xvsdk_INCLUDE} )


# includes
include_directories(${ADDITIONAL_INCLUDE_DIRS})
if(WIN32)
  include_directories(${GLM_INCLUDE_DIRS})
  FIND_PATH(DX_INCLUDE_PATH d3d11.h      PATHS "$ENV{DXSDK_DIR}/Include" "C:/Program Files (x86)/Microsoft DirectX SDK/Include" DOC "The directory where D3D11.h resides")
  find_library(D3D11_LIBRARY d3d11.lib   PATHS "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64" DOC "The directory where d3d11.lib resides")
  find_library(D3DX11_LIBRARY d3dx11.lib PATHS "$ENV{DXSDK_DIR}/Lib/x64" "C:/Program Files (x86)/Microsoft DirectX SDK/Lib/x64" DOC "The directory where d3dx11.lib resides")
  set (D3D11_FOUND "YES")

endif()

set(D3D11_LIBRARIES ${D3D11_LIB})
mark_as_advanced(D3D11_INCLUDE_PATH D3D11_LIB)
#add_subdirectory(./src/Renderer/)
include_directories(./include/)

if(WIN32)  # Windows
  set(CMAKE_EXE_LINKER_FLAGS "-static")
  #the secret sensor
  add_library(libProjectEskyLLAPIX SHARED
    ./src/Tracker/X/main.cpp
    ./src/Tracker/X/Tracker.h
    ./include/DollaryDooFilter.h)
  set_target_properties(libProjectEskyLLAPIX PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyLLAPIX ${DEPENDENCIES} ${xvsdk_LIBRARIES} ${OpenCV_LIBS})  
else()  # Linux
  add_library(libProjectEskyLLAPIIntel SHARED
    ./src/Tracker/IntelT265/main.cpp
    ./src/Tracker/IntelT265/Tracker.h
    ./src/Tracker/IntelT265/LinuxSerialPort.cpp
    ./include/LinuxSerialPort.hpp
    ./include/LinuxSerialException.hpp)
  set_target_properties(libProjectEskyLLAPIIntel PROPERTIES COMPILE_DEFINITIONS BUILDER_STATIC_DEFINE)
  target_link_libraries(libProjectEskyLLAPIIntel ${DEPENDENCIES} ${REALSENSE2_FOUND}  ${OpenCV_LIBS})
endif()