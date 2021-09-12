#include "EskyRenderer_Vulkan.h"

#include "EskyWindow_Vulkan.h"

EskyRenderer_Vulkan::EskyRenderer_Vulkan(DebugCallback debug_callback)
    : EskyRenderer(debug_callback) {
  debugMessage("Initializing Vulkan backend");
}

EskyRenderer_Vulkan::~EskyRenderer_Vulkan() {
  debugMessage("Cleaning up Vulkan backend");
}

void EskyRenderer_Vulkan::processDeviceEvent(UnityGfxDeviceEventType,
                                             IUnityInterfaces*) {}

EskyWindow* EskyRenderer_Vulkan::createWindow(SDL_Window* window_handle) {
  return new EskyWindow_Vulkan(window_handle, this);
}

void EskyRenderer_Vulkan::renderFrame() {}
