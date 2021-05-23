#include "EskyRenderer.h"

#if SUPPORT_D3D11
#include "DirectX/EskyRenderer_D3D11.h"
#endif

#if SUPPORT_VULKAN
#include "Vulkan/EskyRenderer_Vulkan.h"
#endif

EskyRenderer* EskyRenderer::Create(UnityGfxRenderer apiType,
                                   DebugCallback debugCallback) {
#if SUPPORT_D3D11
  if (apiType == kUnityGfxRendererD3D11) {
    return new EskyRenderer_D3D11();
  }
#endif  // if SUPPORT_D3D11

#if SUPPORT_VULKAN
  if (apiType == kUnityGfxRendererVulkan) {
    return new EskyRenderer_Vulkan(debugCallback);
  }
#endif  // if SUPPORT_VULKAN

  // Unknown or unsupported graphics API
  return nullptr;
}

EskyRenderer::EskyRenderer(DebugCallback debug_callback)
    : _debug_callback(debug_callback) {
  debugMessage("Creating EskyRenderer");
}

EskyRenderer::~EskyRenderer() { debugMessage("Destroying EskyRenderer"); }

void EskyRenderer::setDebugCallback(DebugCallback debug_callback) {
  _debug_callback = debug_callback;
}

void EskyRenderer::debugMessage(const char* message) {
  if (_debug_callback != nullptr) {
    _debug_callback(message);
  }
}
