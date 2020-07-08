#include "common_header.h"
#include <windows.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <GL/freeglut.h>
#include <GL/freeglut_ext.h>
#include <GL/wglew.h>
#include <iostream>
#include "IUnityInterface.h"
#include "IUnityGraphics.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include "win_OpenGLApp.h"


HGLRC unityContext;
HGLRC nativeContext;
#define STB_IMAGE_IMPLEMENTATION
float*  left_uv_to_rect_x = new float[16]{-0.7530364531010308, 0.8806592947908687, -0.8357813137161849, 0.3013989721607643, 0.9991764544369446, -0.2578159567698274, 0.3278667335649757, -0.4602577277109663, -0.23980700925448195, -0.056891370605734376, -0.1248008903440144, 0.7641381600051023, 0.20935445281014292, -0.06256983016261788, 0.25844580123833516, -0.5098143951663658};
float*  left_uv_to_rect_y = new float[16]{ 0.5612597403791647, -1.1899589356849427, 0.4652815794139322, -0.2737933233160801, 0.3774305703820774, -0.8110333901413378, 1.2705775357104372, -0.7461290557575936, -0.19222925521894155, 0.936404121235537, -1.7109388784623627, 0.9147182510080394, 0.33073407860855586, -1.1463700238163494, 1.4965795269835196, -0.7090919632511286};
float* right_uv_to_rect_x = new float[16]{ -0.2117125319456463, -0.432262579698108, 0.41675063901331316, -0.14650788483832153, 1.0941580384494245, -0.30628109185189906, 0.109119134429531, 0.11642874201014344, -0.2761527408488216, -0.4335709010559027, 0.9626491769528918, -0.5572405188216735, 0.18342869894719088, 0.37981945016058366, -0.8718621504058989, 0.5218968716935535};
float* right_uv_to_rect_y = new float[16]{ 1.0129568069314265, -2.110976542118192, 1.4108474581893895, -0.7746290913232183, -0.746419837008027, 1.747642287758405, -1.5753294007072252, 0.7143402603200871, 0.5607717274125551, -1.5019493985594772, 1.2539128525783017, -0.42999735712430215, -0.21517910830152714, 0.5965062719847273, -0.5664205050494074, 0.18545738302854597};





//-------------------------------------------------------------------

#define DLL_EXPORT __declspec(dllexport)


#ifdef __cplusplus  
extern "C" {


    int TextureIDLeft = 0;
    int TextureIDRight = 0;
    //A triangle
    bool shouldPushUpdate = false;

    GLuint LoadTexture(const char* filename)
    {
        GLuint texture;
        int width, height, nrChannels;
        unsigned char* data = stbi_load("container.jpg", &width, &height, &nrChannels, 0);

        if (data != nullptr)
        {
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_2D, texture);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            glGenerateMipmap(GL_TEXTURE_2D);
            stbi_image_free(data);
            Debug::Log("Loaded Texture successfully", Color::Green);

        }
        else {
            Debug::Log("Failed to load texture", Color::Red);
        }

        return texture;
    }
#endif
    int WindowID = 0;
    std::thread* t1;
    std::thread* t2;
    int x, y, width, height = 0;
    bool firstEvent = false;
    // Plugin function to handle a specific rendering event
    static void UNITY_INTERFACE_API OnRenderEventObtainUnityContext(int eventID)
    {
        unityContext = wglGetCurrentContext();
    }

    bool doExit = false;
    bool doExit2 = false;
    bool startFunctioning = false;

    void DoFunction2()
    {
        doExit = false;
        doExit2 = false;
        Debug::Log("Creating Window");
        if(!appMain.InitializeApp("ProjectCupboard")) {
            Debug::Log("Unable to init");
        }
        else {
            appMain.RegisterAppClass(NULL);

            if (!appMain.CreateAppWindow("OpenGLRenderWindow")) {
                Debug::Log("Couldn't create the window");
            }
            appMain.ResetTimer();
            appMain.AppBody();
            appMain.Shutdown();

        }        
    }
    DLL_EXPORT void stop() {
        doExit2 = true;
        doExit = true;
        appMain.StopInstance();
    }
    DLL_EXPORT void addLeftRightPointers(int LeftID, int RightID) {
        TextureIDLeft = LeftID;
        TextureIDRight = RightID;
        Debug::Log("Adding the left and right pointers");
    }
    DLL_EXPORT void initialize(int xPos, int yPos, int w, int h) {
        x = xPos;
        y = yPos;
        width = w;
        height = h;
        t1 = new std::thread(DoFunction2);
    }
    extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API
        GetUnityContext()
    {
        return OnRenderEventObtainUnityContext;
    }
    typedef void(*FuncCallBack)(const char* message, int color, int size);
    static FuncCallBack callbackInstance = nullptr;
    DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb);
}
void Debug::Log(const char* message, Color color) {
    if (callbackInstance != nullptr)
        callbackInstance(message, (int)color, (int)strlen(message));
}

void  Debug::Log(const std::string message, Color color) {
    const char* tmsg = message.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}

void  Debug::Log(const int message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const char message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const float message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void  Debug::Log(const double message, Color color) {
    std::stringstream ss;
    ss << message;
    send_log(ss, color);
}

void Debug::Log(const bool message, Color color) {
    std::stringstream ss;
    if (message)
        ss << "true";
    else
        ss << "false";

    send_log(ss, color);
}

void Debug::send_log(const std::stringstream& ss, const Color& color) {
    const std::string tmp = ss.str();
    const char* tmsg = tmp.c_str();
    if (callbackInstance != nullptr)
        callbackInstance(tmsg, (int)color, (int)strlen(tmsg));
}

//Create a callback delegate
void RegisterDebugCallback(FuncCallBack cb) {
    callbackInstance = cb;
}
