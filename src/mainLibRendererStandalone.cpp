#include "common_header.h"
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
#include <array>
#include <cmath>
#include <iostream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>
#include <float.h>
#define STB_IMAGE_IMPLEMENTATION






//-------------------------------------------------------------------

#define DLL_EXPORT __declspec(dllexport)


#ifdef __cplusplus  
extern "C" {


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
    int x, y, width, height = 0;
    bool firstEvent = false; 
    // Plugin function to handle a specific rendering event
    static void UNITY_INTERFACE_API OnRenderEventObtainUnityContext(int eventID)
    {
        appMain.oglControl.unityContext = wglGetCurrentContext();
    }

    bool doExit = false;
    bool startFunctioning = false;
    //realsense stuffs <TODO: Tidy this up later>
   

    void DoFunction2()
    {
        doExit = false;
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
    //ZedTrackerObject* toz; 
    DLL_EXPORT void stop() {

        appMain.StopInstance();
    }

    DLL_EXPORT void SetEyeBorders(float* leftBorders, float* rightborders) {
        appMain.oglControl.LeftEyeBorderConstraints = leftBorders;
        appMain.oglControl.RightEyeBorderConstraints = rightborders;
        appMain.oglControl.updateBorders = true;
        Debug::Log("Updating Borders");
    }
    DLL_EXPORT void setScreenSpaceOffset(float* leftOffset, float* rightOffset) { 
        appMain.oglControl.left_offset_x_y = leftOffset;
        appMain.oglControl.right_offset_x_y = rightOffset;
        appMain.oglControl.updateOffset_x_y = true;
    }
    DLL_EXPORT void setLeftRightCameraMatricies(float* leftCameraMatrix, float* rightCameraMatrix) {
        appMain.oglControl.CameraMatrixLeft = leftCameraMatrix; 
        appMain.oglControl.CameraMatrixRight = rightCameraMatrix;
        appMain.oglControl.useCameraMatricies = true;
    }
    DLL_EXPORT void setLeftRightPointers(int LeftID, int RightID) {        
        appMain.oglControl.TextureIDLeft = LeftID;
        appMain.oglControl.TextureIDRight = RightID;
        appMain.LeftTexture = LeftID;
        appMain.RightTexture = RightID;
        Debug::Log("Adding the left and right pointers ");
    }
     
    DLL_EXPORT void setCalibration(float* leftuvtorectx, float* leftuvtorecty, float* rightuvtorectx, float* rightuvtorecty) {
        appMain.oglControl.left_uv_to_rect_x = leftuvtorectx;
        appMain.oglControl.left_uv_to_rect_y = leftuvtorecty; 
        appMain.oglControl.right_uv_to_rect_x = rightuvtorectx; 
        appMain.oglControl.right_uv_to_rect_y = rightuvtorecty;
    } 
    DLL_EXPORT void initialize(int xPos, int yPos, int w, int h) { 
        x = xPos;
        y = yPos;  
        width = w; 
        height = h; 
        appMain.x = x; 
        appMain.y = y;  
        appMain.width = width; 
        appMain.height = height;
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
