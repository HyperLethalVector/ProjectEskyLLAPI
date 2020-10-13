#include "common_header.h"
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <thread>
#include <iostream>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
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
#include "Tracker.cpp"

#define STB_IMAGE_IMPLEMENTATION






//-------------------------------------------------------------------

#define DLL_EXPORT __declspec(dllexport)
#ifdef __cplusplus  
extern "C" {

#endif
    bool doExit2 = false; 
    TrackerObject* to;
    DLL_EXPORT void StopTrackers() {
        doExit2 = true; 
        if (to != nullptr) {
            Debug::Log("Stopping Realsense Tracking");
            to->DoExit3 = true; 
            to->StopTracking();
        }
    }
    std::thread* t3;
    void DoFunction3() {
        to->DoFunctionTracking();
        delete to;
        to = nullptr;
    }

    void DoFunction4() {
        //toz->DoFunctionTracking();
//        toz->~ZedTrackerObject();
    }
    DLL_EXPORT void StartTrackerThread(bool useLocalization) {//ignored for now....
        Debug::Log("Started Tracking Thread");
        to->DoExit3 = false;
        t3 = new std::thread(DoFunction3);
    }
    DLL_EXPORT void StartTrackerThreadZed(bool useLocalization) {//ignored for now....

        //toz->DoExit3 = false;
        t3 = new std::thread(DoFunction4);
    }
    DLL_EXPORT float* GetLatestPose() { 
        return to->pose;
    }
    DLL_EXPORT void InitializeTrackerObject() {
        if (to == nullptr) { 
            to = new TrackerObject();
        } 
    }
    DLL_EXPORT void ObtainMap() {
        if (to != nullptr) {
            to->GrabMap();
        }
    }
    DLL_EXPORT void SetMapData(unsigned char* inputData, int Length) { 
        if (to != nullptr) {
            Debug::Log("Should Start Now");
            to->ImportMap(inputData, Length); 

        }
    } 
    DLL_EXPORT void ObtainObjectPoseInLocalizedMap(const char* objectID) {
        if (to != nullptr) {
            to->GrabObjectPose(objectID);
        }
        else {
            Debug::Log("WARNING: Tracker not initialized", Color::Yellow);
            nullptr;
        }
    }
    DLL_EXPORT void SetObjectPoseInLocalizedMap(const char* objectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw) {
        if (to != nullptr) {
            Debug::Log("Trying to do the object pose saving", Color::Yellow);
            to->SetObjectPose(objectID,tx,ty,tz,qx,qy,qz,qw);
        }
        else {
            Debug::Log("WARNING: Tracker not initialized", Color::Yellow);
        } 
    }
    DLL_EXPORT void ZedInitializeTrackerObject() {

    }
     
    typedef void(*FuncCallBack)(const char* message, int color, int size);
    static FuncCallBack callbackInstance = nullptr;
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData,int Length);
    typedef void(*FuncCallBack4)(string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
    DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb);
    DLL_EXPORT void RegisterLocalizationCallback(FuncCallBack2 cb);
    DLL_EXPORT void RegisterBinaryMapCallback(FuncCallBack3 cb);
    DLL_EXPORT void RegisterObjectPoseCallback(FuncCallBack4 cb);
    DLL_EXPORT void SaveOriginPose() {
        to->SetOrigin();  
    }
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
void RegisterLocalizationCallback(FuncCallBack2 cb) { 
    if(to != nullptr) 
    to->callbackLocalization = cb;
}
void RegisterObjectPoseCallback(FuncCallBack4 cb) {
    if (to != nullptr) { 
        to->callbackObjectPoseReceived = cb;
    }
}
void RegisterBinaryMapCallback(FuncCallBack3 cb) {
    if (to != nullptr)
        to->callbackBinaryMap = cb;
}
