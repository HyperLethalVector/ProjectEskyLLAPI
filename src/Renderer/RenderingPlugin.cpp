#include "EskyDisplay.h"
#include "EskyRenderer.h"
#include "IUnityGraphics.h"
#include "IUnityInterface.h"

static void OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

// Shared structures
static IUnityInterfaces* s_UnityInterfaces = nullptr;
static IUnityGraphics* s_Graphics = nullptr;
static EskyDisplay* s_CurrentDisplay = nullptr;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;
DebugCallback s_debug = nullptr;

// Internal helper functions
static void DebugMessage(const char* message) {
  if (s_debug) {
    s_debug(message);
  }
}

// Unity exported functions
ESKY_EXPORT void UnityPluginLoad(IUnityInterfaces* unityInterfaces) {
  s_UnityInterfaces = unityInterfaces;
  s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
  s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);

  // Run OnGraphicsDeviceEvent(initialize) manually on plugin load
  OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
}

ESKY_EXPORT void UnityPluginUnload() {
  s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
}

static void UNITY_INTERFACE_API
OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType) {
  // Create graphics API implementation upon initialization
  if (eventType == kUnityGfxDeviceEventInitialize) {
    if (s_CurrentDisplay != nullptr) {
      DebugMessage("Attempted to initialize EskyDisplay twice");
    } else {
      DebugMessage("Initializing EskyRenderer");
      s_DeviceType = s_Graphics->GetRenderer();
      // TODO(marceline-cramer) Pass correct device type to constructor
      s_CurrentDisplay = new EskyDisplay;

      if (s_CurrentDisplay == nullptr) {
        DebugMessage("Failed to create EskyDisplay!");
      }
    }
  }

  // Let the implementation process the device related events
  if (s_CurrentDisplay) {
    DebugMessage("Delegating graphics device event to EskyRenderer");
    s_CurrentDisplay->getRenderer()->processDeviceEvent(eventType,
                                                        s_UnityInterfaces);
  }

  // Cleanup graphics API implementation upon shutdown
  if (eventType == kUnityGfxDeviceEventShutdown) {
    if (s_CurrentDisplay != nullptr) {
      DebugMessage("Destroying EskyDisplay");
      delete s_CurrentDisplay;
      s_CurrentDisplay = nullptr;
      s_DeviceType = kUnityGfxRendererNull;
    } else {
      DebugMessage("Attempted to destroy EskyDisplay twice");
    }
  }
}

// Exported plugin bindings
ESKY_EXPORT void SetRenderTextureWidthHeight(int id, int width, int height) {}

ESKY_EXPORT void SetDebugFunction(void* fp) {
  s_debug = reinterpret_cast<DebugCallback>(fp);
  DebugMessage("Set the debug function");
  // TODO(marceline-cramer): Pass debug callback to s_CurrentRenderer too
}

ESKY_EXPORT void StartWindowById(int windowId, const wchar_t* title, int width,
                                 int height, bool noBorder) {}

ESKY_EXPORT void StopWindowById(int windowId) {}

ESKY_EXPORT void SetWindowRectById(int windowId, int left, int top, int width,
                                   int height) {}

ESKY_EXPORT void SendTextureIdToPluginByIdLeft(int windowId, void* texturePtr) {
}

ESKY_EXPORT void SendTextureIdToPluginByIdRight(int windowId,
                                                void* texturePtr) {}

ESKY_EXPORT void SetColorFormat(int colorFormat) {}

ESKY_EXPORT void SetDeltas(int windowId, void* deltaLeft, void* deltaRight) {}

ESKY_EXPORT UnityRenderingEvent InitGraphics() {}

ESKY_EXPORT UnityRenderingEvent GetRenderEventFunc() {}

ESKY_EXPORT void SetEnableFlagWarping(int id, bool enabled) {}

ESKY_EXPORT void SetRequiredValuesById(int windowId, ...) {}

ESKY_EXPORT void SetBrightness(int id, float brightness) {}

// TODO(marceline-cramer) Implement this
// ESKY_EXPORT void SetQualitySettings(int count, int quality) {}
