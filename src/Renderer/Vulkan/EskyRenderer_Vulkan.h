#pragma once

#include "EskyRenderer.h"

#ifdef SUPPORT_VULKAN

class EskyRenderer_Vulkan : public EskyRenderer {
 public:
  ~EskyRenderer_Vulkan() {}

  void ProcessDeviceEvent(UnityGfxDeviceEventType, IUnityInterfaces*) final {}

 private:
};

#endif  // #ifdef SUPPORT_VULKAN
