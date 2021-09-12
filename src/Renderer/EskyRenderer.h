#pragma once

#include "EskyWindow.h"
#include "IUnityGraphics.h"

// Shorthand macro for exports
#define ESKY_EXPORT extern "C" UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API

#define SUPPPORT_VULKAN

using DebugCallback = void (*)(const char*);

class EskyRenderer {
 public:
  static EskyRenderer* Create(UnityGfxRenderer, DebugCallback);

  EskyRenderer(DebugCallback);
  virtual ~EskyRenderer();

  virtual void processDeviceEvent(UnityGfxDeviceEventType,
                                  IUnityInterfaces*) = 0;
  virtual EskyWindow* createWindow(SDL_Window*) = 0;
  virtual void renderFrame() = 0;

  void setDebugCallback(DebugCallback);
  void debugMessage(const char*);

 private:
  DebugCallback _debug_callback;
};
