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
#include "Tracker_ZED.cpp"

#define STB_IMAGE_IMPLEMENTATION
//-------------------------------------------------------------------
#define DLL_EXPORT __declspec(dllexport)

#ifdef __cplusplus  
extern "C" {


#endif
    TrackerObject* to;
    ID3D11Device* m_Device;
    DLL_EXPORT void StopTrackers() {
        if (to != nullptr) {
            Debug::Log("Stopping ZED Tracking");
            to->ExitThreadLoop = true;
        }
    } 
    std::thread* trackerThread;
    void DoFunction3() {
        to->DoFunctionTracking();
        delete to;
        to = nullptr;
    }
    DLL_EXPORT void StartTrackerThread(bool useLocalization) {//ignored for now....
        Debug::Log("Started Tracking Thread");
        to->ExitThreadLoop = false; 
        trackerThread = new std::thread(DoFunction3);
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
    DLL_EXPORT void RegisterMeshCallback(TrackerObject::FuncMeshCallback myCallback) {
        if (to != nullptr) {
            to->meshCallback = myCallback;
        }
    }
    DLL_EXPORT void RegisterMeshCompleteCallback(TrackerObject::FuncMeshCompleteCallback myCallback) { 
        if (to != nullptr) {
            to->meshCompleteCallback = myCallback;
        }
    }
    static IUnityInterfaces* s_UnityInterfaces = NULL;
    static IUnityGraphics* s_Graphics = NULL;

    static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType);
    extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces * unityInterfaces)
    {
        s_UnityInterfaces = unityInterfaces;
        s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
        s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent); 
        OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize);
    }

    extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
    {
        s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
    }
     
    DLL_EXPORT void ProcessDeviceEvent(UnityGfxDeviceEventType type, IUnityInterfaces* interfaces)
    {
        
    }
    typedef void(*FuncTextureInitializedCallback)(int TextureWidth, int TextureHeight, int textureCount, float v_fov);
    DLL_EXPORT void SetTextureInitializedCallback(FuncTextureInitializedCallback myCallback) {
        if (to != nullptr) {
            to->textureInitializedCallback = myCallback;
        }
        else {
            Debug::Log("Tracker not initialized!!");
        }
    }
    static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
    {
        // Create graphics API implementation upon initialization
        if (eventType == kUnityGfxDeviceEventInitialize)
        {
            IUnityGraphicsD3D11* d3d = s_UnityInterfaces->Get<IUnityGraphicsD3D11>();
            m_Device = d3d->GetDevice();
        }
        else if (eventType == kUnityGfxDeviceEventShutdown) {
        }       
    }
    
    DLL_EXPORT void HookDeviceToZed() {
        if (to != nullptr) {
            to->m_Device = m_Device; 
        } 
        else {
            Debug::Log("Tracker hasn't been initialized", Color::Red); 
        }   
    }
    DLL_EXPORT void SetRenderTexturePointer(void* textureHandle) {
        if (to != nullptr) {
            to->SetTexturePointer(textureHandle);
        }
    }
    DLL_EXPORT void StartSpatialMapping(int ChunkSizes) {
        if (to != nullptr) {
            to->StartSpatialMapping(ChunkSizes); 
        }
    } 
    DLL_EXPORT void StopSpatialMapping(int ChunkSizes) {
        if (to != nullptr) {
            to->StopSpatialMapping();
        }
    }
    DLL_EXPORT void CompletedMeshUpdate() {
        if (to != nullptr) {
            to->shouldRequestNewChunks = true;
        }
    }
}
static void UNITY_INTERFACE_API OnRenderEvent(int eventID)  
{
    if (to != nullptr) {
        to->UpdatecameraTextureGPU();
    }
}

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc()
{
    return OnRenderEvent;
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
  