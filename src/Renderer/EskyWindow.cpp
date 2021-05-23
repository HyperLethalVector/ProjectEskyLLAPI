#include "EskyWindow.h"

EskyWindow::EskyWindow(SDL_Window* window_handle)
    : _window_handle(window_handle) {}

EskyWindow::~EskyWindow() { SDL_DestroyWindow(_window_handle); }

void EskyWindow::setTitle(const wchar_t*) {}

void EskyWindow::setRect(int, int, int, int) {}
