#pragma once

#include <SDL2/SDL.h>

#include <vector>

// Forward declarations
class EskyRenderer;
class EskyWindow;

class EskyDisplay {
 public:
  // TODO(marceline-cramer) More constructors w/ callback, Unity init info, etc.
  EskyDisplay();
  ~EskyDisplay();

  void run();

  EskyRenderer* getRenderer();

  void startWindowById(int, const wchar_t*, int, int, bool);
  void stopWindowById(int);

  void setColorFormat(int);
  void setQualitySettings(int count, int quality);

 private:
  EskyRenderer* _renderer;

  using Window = SDL_Window*;
  std::vector<Window> _sdl_windows;

  // Private helper methods
  Window _assertWindow(int);
  static void _debugCallback(const char*);
};
