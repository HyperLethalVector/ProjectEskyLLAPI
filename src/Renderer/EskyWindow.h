#pragma once

#include <SDL2/SDL.h>

class EskyWindow {
 public:
  EskyWindow(SDL_Window*);
  virtual ~EskyWindow();

  void onEvent(const SDL_WindowEvent*);
  void setTitle(const wchar_t*);
  void setRect(int, int, int, int);
  SDL_Window* getHandle();

  virtual void setDeltas(void*, void*) = 0;
  virtual void setRequiredValues(...) = 0;
  virtual void setRenderTextureWidthHeight(int, int) = 0;
  virtual void setEnableFlagWarping(bool) = 0;
  virtual void setBrightness(float) = 0;
  virtual void sendTextureIdToPluginByIdLeft(void*) = 0;
  virtual void sendTextureIdToPluginByIdRight(void*) = 0;

 protected:
  virtual void _onResize(int, int) = 0;
  SDL_Window* const _window_handle;
};
