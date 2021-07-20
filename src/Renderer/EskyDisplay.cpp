#include "EskyDisplay.h"

#include <iostream>
#include <sstream>

#include "EskyRenderer.h"

EskyDisplay::EskyDisplay() {
  _debugCallback("Creating EskyDisplay");

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    std::ostringstream error_message;
    error_message << "Failed to initialize SDL: ";
    error_message << SDL_GetError();
    _debugCallback(error_message.str().c_str());
    return;
  }

  _renderer = EskyRenderer::Create(kUnityGfxRendererVulkan, _debugCallback);
}

EskyDisplay::~EskyDisplay() {
  _debugCallback("Destroying EskyDisplay");

  for (int i = 0; i < _windows.size(); i++) {
    stopWindowById(i);
  }

  if (_renderer) {
    delete _renderer;
  }

  SDL_Quit();
}

void EskyDisplay::run() {
  bool quit = false;
  while (!quit) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      switch (e.type) {
        case SDL_QUIT: {
          quit = true;
          break;
        }

        case SDL_WINDOWEVENT: {
          SDL_Window* event_window = SDL_GetWindowFromID(e.window.windowID);
          for (auto& window : _windows) {
            if (window && event_window == window->getHandle()) {
              window->onEvent(&e.window);
              break;
            }
          }

          break;
        }

        default:
          break;
      }
    }

    _renderer->renderFrame();
  }
}

EskyRenderer* EskyDisplay::getRenderer() { return _renderer; }

void EskyDisplay::startWindowById(int id, const wchar_t*, int, int, bool) {
  // Check that there isn't already a window here
  EskyWindow* old_window = _assertWindow(id);
  if (old_window) {
    std::ostringstream error_message;
    error_message << "Window #" << id << " is already started";
    _debugCallback(error_message.str().c_str());
    return;
  }

  SDL_Window* new_window = SDL_CreateWindow(
      "Esky Window", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 800, 600,
      SDL_WINDOW_SHOWN | SDL_WINDOW_VULKAN);

  if (!new_window) {
    std::ostringstream error_message;
    error_message << "Failed to create SDL window: ";
    error_message << SDL_GetError();
    _debugCallback(error_message.str().c_str());
    return;
  }

  // Ensure that the window vector has enough room
  if (id >= _windows.size()) {
    // Pad new elements with nullptr
    _windows.resize(id + 1, nullptr);
  }

  _windows[id] = _renderer->createWindow(new_window);
}

EskyWindow* EskyDisplay::getWindowById(int id) { return _assertWindow(id); }

void EskyDisplay::stopWindowById(int id) {
  EskyWindow* window = _assertWindow(id);

  if (window) {
    _windows[id] = nullptr;
    delete window;
  }
}

void EskyDisplay::setColorFormat(int colorFormat) {
  // TODO(marceline-cramer) Set color format
}

EskyWindow* EskyDisplay::_assertWindow(int id) {
  // TODO(marceline-cramer) DebugMessage here
  if (id < 0 || id >= _windows.size()) {
    return nullptr;
  }

  EskyWindow* window = _windows[id];
  return window;
}

void EskyDisplay::_debugCallback(const char* message) {
  std::cerr << "Debug: " << message << std::endl;
}
