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
  EskyWindow* getWindowById(int);
  void stopWindowById(int);

  void setColorFormat(int);
  void setQualitySettings(int count, int quality);

 private:
  EskyRenderer* _renderer;

  std::vector<EskyWindow*> _windows;

  // Private helper methods
  EskyWindow* _assertWindow(int);
  static void _debugCallback(const char*);
};
