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
#define Math_PI 3.14159265
using namespace std;
// Some useful defines

#define FOR(q,n) for(int q=0;q<n;q++)
#define SFOR(q,s,e) for(int q=s;q<=e;q++)
#define RFOR(q,n) for(int q=n;q>=0;q--)
#define RSFOR(q,s,e) for(int q=s;q>=e;q--)
typedef void(*FuncCallBack)(const char* message, int color, int size);
static FuncCallBack callbackInstance = nullptr;
#define ESZ(elem) (int)elem.size()
enum class Color { Red, Green, Blue, Black, White, Yellow, Orange };
#ifndef common_header
#define common_header
#endif
class  Debug
{
private:
    static void send_log(const std::stringstream& ss, const Color& color) {
        const std::string tmp = ss.str();
        const char* tmsg = tmp.c_str();
        if (callbackInstance != nullptr)
            callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
    }
public:
    static void Log(const char* message, Color color = Color::Black) {
        if (callbackInstance != nullptr)
            callbackInstance(message, (int)color, (int)strlen(message));
    }
    static void Log(const std::string message, Color color = Color::Black) {
        const char* tmsg = message.c_str();
        if (callbackInstance != nullptr)
            callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
    }
    static void Log(const int message, Color color = Color::Black) {
        std::stringstream ss;
        ss << message;
        send_log(ss, color);
    }
    static void Log(const char message, Color color = Color::Black) {
        std::stringstream ss;
        ss << message;
        send_log(ss, color);
    }
    static void Log(const float message, Color color = Color::Black) {
        std::stringstream ss;
        ss << message;
        send_log(ss, color);
    }
    static void Log(const double message, Color color = Color::Black) {
        std::stringstream ss;
        ss << message;
        send_log(ss, color);
    }
    static void Log(const bool message, Color color = Color::Black) {
        std::stringstream ss;
        if (message)
            ss << "true";
        else
            ss << "false";
        send_log(ss, color);
    }


};