# Accumulate dependencies
set(RENDERING_PLUGIN_SRC
  ./EskyDisplay.cpp
  ./EskyRenderer.cpp
  ./EskyWindow.cpp
  ./RenderingPlugin.cpp)

set(RENDERING_PLUGIN_DEFS
  BUILDER_STATIC_DEFINE)

set(RENDERING_PLUGIN_LIB)

set(RENDERING_PLUGIN_INCLUDE
  # Include files at root
  ${CMAKE_CURRENT_SOURCE_DIR})

# Only build rendering plugin if at least one graphics API is found
set(BUILD_RENDERING_PLUGIN NO)

# Vulkan
include(FindVulkan)

if(Vulkan_FOUND)
  message("Building rendering plugin with Vulkan support")
  set(BUILD_RENDERING_PLUGIN YES)
  list(APPEND RENDERING_PLUGIN_SRC
    ./Vulkan/EskyRenderer_Vulkan.cpp
    ./Vulkan/EskyWindow_Vulkan.cpp)
  list(APPEND RENDERING_PLUGIN_DEFS "SUPPORT_VULKAN")

  # Use SDL only for Vulkan for now, until we've refactored the renderer
  find_package(SDL2 REQUIRED COMPONENTS SDL2::SDL2)
  list(APPEND RENDERING_PLUGIN_LIB SDL2::SDL2)
else()
  message(WARNING "Building rendering plugin without Vulkan support")
endif()

if(D3D11_FOUND STREQUAL "YES")
  message("Building rendering plugin with D3D11 support")
  set(BUILD_RENDERING_PLUGIN YES)

  list(APPEND RENDERING_PLUGIN_SRC
    ./DirectX/main.cpp
    ./DirectX/graphics.cpp
    ./DirectX/graphics.h)

  list(APPEND RENDERING_PLUGIN_DEFS "SUPPORT_D3D11")
elseif(WIN32)
  message(WARNING "Building rendering plugin without D3D11 support")
endif()

if(BUILD_RENDERING_PLUGIN)
  # C++14 is required for <memory>
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)

  add_library(ProjectEskyLLAPIRenderer SHARED ${RENDERING_PLUGIN_SRC})
  target_include_directories(ProjectEskyLLAPIRenderer PUBLIC ${RENDERING_PLUGIN_INCLUDE})
  target_compile_definitions(ProjectEskyLLAPIRenderer PUBLIC ${RENDERING_PLUGIN_DEFS})
  target_link_libraries(ProjectEskyLLAPIRenderer PUBLIC ${RENDERING_PLUGIN_LIB})

  # This shared library doesn't need to link with any graphics libraries
  #   because the Unity runtime provides the symbols for those

  if(BUILD_RENDERER_TEST)
    add_executable(renderer-test ./test.cpp)
    target_include_directories(renderer-test PUBLIC ${RENDERING_PLUGIN_INCLUDE})
    target_link_libraries(renderer-test ProjectEskyLLAPIRenderer)
  endif()
else()
  message(WARNING "Not building rendering plugin; no graphics APIs found")
endif()
