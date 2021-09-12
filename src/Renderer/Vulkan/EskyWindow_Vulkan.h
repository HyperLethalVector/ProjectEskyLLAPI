#pragma once

#include "EskyWindow.h"

// Forward declarations
class EskyRenderer_Vulkan;

class EskyWindow_Vulkan : public EskyWindow {
 public:
  EskyWindow_Vulkan(SDL_Window*, EskyRenderer_Vulkan*);

  // EskyWindow implementation
  void setDeltas(void*, void*) final;
  void setRequiredValues(...) final;
  void setRenderTextureWidthHeight(int, int) final;
  void setEnableFlagWarping(bool) final;
  void setBrightness(float) final;
  void sendTextureIdToPluginByIdLeft(void*) final;
  void sendTextureIdToPluginByIdRight(void*) final;

 protected:
  // EskyWindow implementation
  void _onResize(int, int) final;

 private:
  EskyRenderer_Vulkan* const _renderer;
};
