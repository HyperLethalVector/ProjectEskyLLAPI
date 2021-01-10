// Header files of frequent usage
#pragma once
#ifdef __linux
#else
#include <windows.h>
#endif
#include <algorithm>
#include <cmath>
#include <cstring>
#include <vector>
#include <sstream>
#include <queue>
#include <map>
#include <set>

using namespace std;

struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double x, y, z;
};
struct Translation {
    double x, y, z;
};
// Some useful defines

#define FOR(q,n) for(int q=0;q<n;q++)
#define SFOR(q,s,e) for(int q=s;q<=e;q++)
#define RFOR(q,n) for(int q=n;q>=0;q--)
#define RSFOR(q,s,e) for(int q=s;q>=e;q--)


#define ESZ(elem) (int)elem.size()
enum class Color { Red, Green, Blue, Black, White, Yellow, Orange };
#ifndef common_header
#define common_header
#endif
class  Debug
{

public:
    static void Log(const char* message, Color color = Color::Black);
    static void Log(const std::string message, Color color = Color::Black);
    static void Log(const int message, Color color = Color::Black);
    static void Log(const char message, Color color = Color::Black);
    static void Log(const float message, Color color = Color::Black);
    static void Log(const double message, Color color = Color::Black);
    static void Log(const bool message, Color color = Color::Black);

private:
    static void send_log(const std::stringstream& ss, const Color& color);
};
