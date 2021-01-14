#include "EskyRenderer.h"

#include "DirectX/EskyRenderer_D3D11.h"
#include "Vulkan/EskyRenderer_Vulkan.h"

EskyRenderer* EskyRenderer::Create(UnityGfxRenderer apiType) {
#if SUPPORT_D3D11
  if (apiType == kUnityGfxRendererD3D11) {
    return new EskyRenderer_D3D11();
  }
#endif  // if SUPPORT_D3D11

#if SUPPORT_VULKAN
  if (apiType == kUnityGfxRendererVulkan) {
    return new EskyRenderer_Vulkan();
  }
#endif  // if SUPPORT_VULKAN

  // Unknown or unsupported graphics API
  return nullptr;
}
