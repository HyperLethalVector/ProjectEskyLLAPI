#include <assert.h>

#include "EskyRenderer.h"
#include "IUnityGraphics.h"
#include "IUnityInterface.h"

static void UNITY_INTERFACE_API
OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

static IUnityInterfaces* s_UnityInterfaces = nullptr;
static IUnityGraphics* s_Graphics = nullptr;

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
UnityPluginLoad(IUnityInterfaces* unityInterfaces) {
  s_UnityInterfaces = unityInterfaces;
  s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
  s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);

#if SUPPORT_VULKAN
  if (s_Graphics->GetRenderer() == kUnityGfxRendererNull) {
    extern void RenderAPI_Vulkan_OnPluginLoad(IUnityInterfaces*);
    RenderAPI_Vulkan_OnPluginLoad(unityInterfaces);
  }
#endif  // SUPPORT_VULKAN

  // Run OnGraphicsDeviceEvent(initialize) manually on plugin load
  OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload() {
  s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
}

static EskyRenderer* s_CurrentRenderer = nullptr;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;

static void UNITY_INTERFACE_API
OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType) {
  // Create graphics API implementation upon initialization
  if (eventType == kUnityGfxDeviceEventInitialize) {
    assert(s_CurrentRenderer == nullptr);
    s_DeviceType = s_Graphics->GetRenderer();
    s_CurrentRenderer = EskyRenderer::Create(s_DeviceType);
  }

  // Let the implementation process the device related events
  if (s_CurrentRenderer) {
    s_CurrentRenderer->ProcessDeviceEvent(eventType, s_UnityInterfaces);
  }

  // Cleanup graphics API implementation upon shutdown
  if (eventType == kUnityGfxDeviceEventShutdown) {
    delete s_CurrentRenderer;
    s_CurrentRenderer = nullptr;
    s_DeviceType = kUnityGfxRendererNull;
  }
}
