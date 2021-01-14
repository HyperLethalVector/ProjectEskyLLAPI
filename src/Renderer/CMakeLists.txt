# Accumulate dependencies
set(RENDERING_PLUGIN_SRC
  ./EskyRenderer.cpp
  ./RenderingPlugin.cpp)

set(RENDERING_PLUGIN_DEFS
  BUILDER_STATIC_DEFINE
)

# Only build rendering plugin if at least one graphics API is found
set(BUILD_RENDERING_PLUGIN NO)

# Include files at root
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Vulkan
include(FindVulkan)

if(Vulkan_FOUND)
  message("Building rendering plugin with Vulkan support")
  set(BUILD_RENDERING_PLUGIN YES)

  list(APPEND RENDERING_PLUGIN_SRC)

  list(APPEND RENDERING_PLUGIN_DEFS "SUPPORT_VULKAN")
else()
  message(WARNING "Building rendering plugin without Vulkan support")
endif()

if(D3D11_FOUND STREQUAL "YES")
  message("Building rendering plugin with D3D11 support")
  set(BUILD_RENDERING_PLUGIN YES)

  list(APPEND RENDERING_PLUGIN_SRC
    ./src/Renderer/DirectX/main.cpp
    ./src/Renderer/DirectX/graphics.cpp
    ./src/Renderer/DirectX/graphics.h)

  list(APPEND RENDERING_PLUGIN_DEFS "SUPPORT_D3D11")
elseif(WIN32)
  message(WARNING "Building rendering plugin without D3D11 support")
endif()

if(BUILD_RENDERING_PLUGIN)
  add_library(ProjectEskyLLAPIRenderer SHARED ${RENDERING_PLUGIN_SRC})
  target_compile_definitions(ProjectEskyLLAPIRenderer PUBLIC ${RENDERING_PLUGIN_DEFS})
  # This shared library doesn't need to link with any graphics libraries
  #   because the Unity runtime provides the symbols for those
else()
  message(WARNING "Not building rendering plugin; no graphics APIs found")
endif()
