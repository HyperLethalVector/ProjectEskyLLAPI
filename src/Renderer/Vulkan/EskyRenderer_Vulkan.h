#pragma once

#include "EskyRenderer.h"

class EskyRenderer_Vulkan : public EskyRenderer {
 public:
  // Take Vulkan context from device in new constructor
  EskyRenderer_Vulkan(DebugCallback);
  ~EskyRenderer_Vulkan();

  // EskyRenderer implementation
  void processDeviceEvent(UnityGfxDeviceEventType, IUnityInterfaces*) final;
  EskyWindow* createWindow(SDL_Window*) final;
  void renderFrame() final;
};
