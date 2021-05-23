// Simple program for testing renderer without having to restart Unity

#include <iostream>
#include <memory>

#include "EskyDisplay.h"

using namespace std;

// Unused for now
// TODO(marceline-cramer) Pass debug callback to EskyDisplay
/*void debugCallback(const char* message) {
  cerr << "Debug: " << message << endl;
}*/

int main() {
  EskyDisplay display;

  display.startWindowById(0, L"Esky Test", 640, 480, false);
  display.run();
  display.stopWindowById(0);

  return 0;
}
