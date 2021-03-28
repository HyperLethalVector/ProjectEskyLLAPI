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

  for (int i = 0; i < _sdl_windows.size(); i++) {
    stopWindowById(i);
  }

  if (_renderer) {
    delete _renderer;
  }

  SDL_Quit();
}

void EskyDisplay::run() {}

EskyRenderer* EskyDisplay::getRenderer() { return _renderer; }

void EskyDisplay::startWindowById(int id, const wchar_t*, int, int, bool) {
  // Check that there isn't already a window here
  Window new_window = _assertWindow(id);
  if (new_window != nullptr) {
    std::ostringstream error_message;
    error_message << "Window #" << id << " is already started";
    _debugCallback(error_message.str().c_str());
    return;
  }

  new_window = SDL_CreateWindow("Esky Window", SDL_WINDOWPOS_UNDEFINED,
                                SDL_WINDOWPOS_UNDEFINED, 800, 600,
                                SDL_WINDOW_SHOWN | SDL_WINDOW_VULKAN);

  if (!new_window) {
    std::ostringstream error_message;
    error_message << "Failed to create SDL window: ";
    error_message << SDL_GetError();
    _debugCallback(error_message.str().c_str());
    return;
  }

  // Ensure that the window vector has enough room
  if (id >= _sdl_windows.size()) {
    // Pad new elements with nullptr
    _sdl_windows.resize(id + 1, nullptr);
  }

  _sdl_windows[id] = new_window;
}

void EskyDisplay::stopWindowById(int id) {
  Window window = _sdl_windows[id];
  _sdl_windows[id] = nullptr;

  if (window != nullptr) {
    SDL_DestroyWindow(window);
  }
}

EskyDisplay::Window EskyDisplay::_assertWindow(int id) {
  // TODO(marceline-cramer) DebugMessage here
  if (id < 0 || id >= _sdl_windows.size()) {
    return nullptr;
  }

  Window window = _sdl_windows[id];
  if (window == nullptr) {
    return nullptr;
  }

  return window;
}

void EskyDisplay::_debugCallback(const char* message) {
  std::cerr << "Debug: " << message << std::endl;
}
