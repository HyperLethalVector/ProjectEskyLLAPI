cmake_minimum_required(VERSION 3.7.2)

# Proeject name - should not contain whitespace 
project(OpengGL_CPP)

#========== Global Configurations =============#
#----------------------------------------------#

# Set the C++ standard for all targets (It sets the flags
# (-std=c++11, -std=c++14 ...) on Clang or GCC. and /std=c++17 on MSVC
# OPTIONAL:
#---------------------------------
set(CMAKE_CXX_STANDARD 17)     
set(CMAKE_VERBOSE_MAKEFILE ON)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
include_directories(${GLEW_INCLUDE_DIR})
include_directories(./include)
include_directories(${GLUT_INCLUDE_DIR})
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})
#========== Targets Configurations ============#

# Build an executable (Unix-like OSes generates ./ProjectCupboardLLAPI, on
# Windows ./ProjectCupboardLLAPI.exe)
# ..........................................
add_library(libProjectCupboardLLAPI SHARED ./src/mainLib.cpp ./src/OpenGLControl.cpp ./src/RenderScene.cpp ./src/Shaders.cpp ./src/Win_OpenGLApp.cpp)

# Set executable dependency libraries
# Equivalent to pass flags -lGL, -lGLU and -lglut 
target_link_libraries(libProjectCupboardLLAPI ${GLUT_LIBRARIES} ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})
set_target_properties(libProjectCupboardLLAPI PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY_DEBUG E:/UnityProjects/NorthStarUnityIntegration/Assets/Plugins/x64
                      RUNTIME_OUTPUT_DIRECTORY_RELEASE E:/UnityProjects/NorthStarUnityIntegration/Assets/Plugins/x64/
)
