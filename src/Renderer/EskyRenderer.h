#pragma once

#include "IUnityGraphics.h"

#define SUPPPORT_VULKAN

class EskyRenderer {
 public:
  static EskyRenderer* Create(UnityGfxRenderer);
  virtual ~EskyRenderer() {}

  virtual void ProcessDeviceEvent(UnityGfxDeviceEventType,
                                  IUnityInterfaces*) = 0;

 private:
};
