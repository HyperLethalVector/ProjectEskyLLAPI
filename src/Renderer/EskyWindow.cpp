#include "EskyWindow.h"

EskyWindow::EskyWindow(SDL_Window* window_handle)
    : _window_handle(window_handle) {}

EskyWindow::~EskyWindow() { SDL_DestroyWindow(_window_handle); }

void EskyWindow::onEvent(const SDL_WindowEvent* event) {
  switch (event->event) {
    case SDL_WINDOWEVENT_SIZE_CHANGED: {
      int width = event->data1;
      int height = event->data2;
      _onResize(width, height);
      break;
    }

    default:
      break;
  }
}

SDL_Window* EskyWindow::getHandle() { return _window_handle; }

void EskyWindow::setTitle(const wchar_t*) {}

void EskyWindow::setRect(int, int, int, int) {}
