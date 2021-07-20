#include <memory>

#include "EskyDisplay.h"
#include "EskyRenderer.h"
#include "EskyWindow.h"
#include "IUnityGraphics.h"
#include "IUnityInterface.h"

static void OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);

// Shared structures
static IUnityInterfaces* s_UnityInterfaces = nullptr;
static IUnityGraphics* s_Graphics = nullptr;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;
static std::unique_ptr<EskyDisplay> s_CurrentDisplay = nullptr;
DebugCallback s_debug = nullptr;

// Internal helper functions
static void s_DebugMessage(const char* message) {
  if (s_debug) {
    s_debug(message);
  }
}

static EskyWindow* s_GetWindow(int id) {
  if (!s_CurrentDisplay) {
    s_DebugMessage("ERROR: Attempted to acquire window from null display");
    return nullptr;
  }

  EskyWindow* window = s_CurrentDisplay->getWindowById(id);

  if (!window) {
    s_DebugMessage("WARNING: Fetching null window from display");
  }

  return window;
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

static void OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType) {
  // Create graphics API implementation upon initialization
  if (eventType == kUnityGfxDeviceEventInitialize) {
    if (s_CurrentDisplay) {
      s_DebugMessage("Attempted to initialize EskyDisplay twice");
    } else {
      s_DebugMessage("Initializing EskyRenderer");
      s_DeviceType = s_Graphics->GetRenderer();
      // TODO(marceline-cramer) Pass correct device type to constructor
      s_CurrentDisplay = std::make_unique<EskyDisplay>();

      if (!s_CurrentDisplay) {
        s_DebugMessage("Failed to create EskyDisplay!");
      }
    }
  }

  // Let the implementation process the device related events
  if (s_CurrentDisplay) {
    s_DebugMessage("Delegating graphics device event to EskyRenderer");
    s_CurrentDisplay->getRenderer()->processDeviceEvent(eventType,
                                                        s_UnityInterfaces);
  }

  // Cleanup graphics API implementation upon shutdown
  if (eventType == kUnityGfxDeviceEventShutdown) {
    if (s_CurrentDisplay) {
      s_DebugMessage("Destroying EskyDisplay");
      delete s_CurrentDisplay.release();
      s_DeviceType = kUnityGfxRendererNull;
    } else {
      s_DebugMessage("Attempted to destroy EskyDisplay twice");
    }
  }
}

// Exported plugin bindings
ESKY_EXPORT void SetRenderTextureWidthHeight(int id, int width, int height) {}

ESKY_EXPORT void SetDebugFunction(void* fp) {
  s_debug = reinterpret_cast<DebugCallback>(fp);
  s_DebugMessage("Set the debug function");
  // TODO(marceline-cramer): Pass debug callback to s_CurrentRenderer too
}

ESKY_EXPORT void StartWindowById(int id, const wchar_t* title, int width,
                                 int height, bool noBorder) {
  s_CurrentDisplay->startWindowById(id, title, width, height, noBorder);
}

ESKY_EXPORT void StopWindowById(int id) {
  s_CurrentDisplay->stopWindowById(id);
}

ESKY_EXPORT void SetWindowRectById(int id, int left, int top, int width,
                                   int height) {
  s_GetWindow(id)->setRect(left, top, width, height);
}

ESKY_EXPORT void SendTextureIdToPluginByIdLeft(int id, void* texturePtr) {
  s_GetWindow(id)->sendTextureIdToPluginByIdRight(texturePtr);
}

ESKY_EXPORT void SendTextureIdToPluginByIdRight(int id, void* texturePtr) {
  s_GetWindow(id)->sendTextureIdToPluginByIdRight(texturePtr);
}

ESKY_EXPORT void SetColorFormat(int colorFormat) {
  s_CurrentDisplay->setColorFormat(colorFormat);
}

ESKY_EXPORT void SetDeltas(int id, void* deltaLeft, void* deltaRight) {
  s_GetWindow(id)->setDeltas(deltaLeft, deltaRight);
}

/*static void OnInitGraphics(int eventID) { s_GetWindow(eventID)->init(); }

ESKY_EXPORT UnityRenderingEvent InitGraphics() { return OnInitGraphics; }*/

static void OnRenderEvent(int eventID) {
  if (!s_CurrentDisplay) {
    s_DebugMessage("ERROR: Attempted to render null display");
    return;
  }

  s_CurrentDisplay->getRenderer()->renderFrame();
}

ESKY_EXPORT UnityRenderingEvent GetRenderEventFunc() { return OnRenderEvent; }

ESKY_EXPORT void SetEnableFlagWarping(int id, bool enabled) {
  s_GetWindow(id)->setEnableFlagWarping(enabled);
}

ESKY_EXPORT void SetRequiredValuesById(int windowId, ...) {}

ESKY_EXPORT void SetBrightness(int id, float brightness) {
  s_GetWindow(id)->setBrightness(brightness);
}

// TODO(marceline-cramer) Implement this
// ESKY_EXPORT void SetQualitySettings(int count, int quality) {}
