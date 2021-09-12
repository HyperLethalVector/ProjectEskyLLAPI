#include "EskyWindow_Vulkan.h"

#include "EskyRenderer_Vulkan.h"

EskyWindow_Vulkan::EskyWindow_Vulkan(SDL_Window* window_handle,
                                     EskyRenderer_Vulkan* renderer)
    : EskyWindow(window_handle), _renderer(renderer) {}

void EskyWindow_Vulkan::setDeltas(void*, void*) {}

void EskyWindow_Vulkan::setRequiredValues(...) {}

void EskyWindow_Vulkan::setRenderTextureWidthHeight(int, int) {}

void EskyWindow_Vulkan::setEnableFlagWarping(bool) {}

void EskyWindow_Vulkan::setBrightness(float) {}

void EskyWindow_Vulkan::sendTextureIdToPluginByIdLeft(void*) {}

void EskyWindow_Vulkan::sendTextureIdToPluginByIdRight(void*) {}

void EskyWindow_Vulkan::_onResize(int, int) {}
