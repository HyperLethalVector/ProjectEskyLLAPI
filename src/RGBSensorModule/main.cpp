#include "common_header.h" 
#include "IUnityInterface.h"
#include "IUnityGraphics.h" 
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
#include "RGBSensorModule.cpp"   
#ifdef __linux__ 
#else 
#include <d3d11.h>   
#define STB_IMAGE_IMPLEMENTATION
#endif 

#ifdef __linux__
#define DLL_EXPORT 
#else
#define DLL_EXPORT __declspec(dllexport) 
#endif 
#ifdef __linux
#else
#endif
ID3D11Device* m_Device;  
static std::map<int, std::thread*> sensorThreads;
static std::map<int, RGBSensorModule*> sensors;  
extern "C" { 
    DLL_EXPORT void StopCameras() {
        std::map<int, RGBSensorModule*>::iterator it = sensors.begin(); 
        while (it != sensors.end()) {
            if (it->second != nullptr) {
                it->second->stopFlags = true; 
                it++;
            } 
        }
        std::map<int, std::thread*>::iterator itt = sensorThreads.begin();
        while (itt != sensorThreads.end()) {
            if (itt->second != nullptr) {
                itt->second->~thread();
                itt++;
            } 
        }
    }    
    void startCameraThread(int i) {  
        sensors[i]->OpenCameraThread(); 
    }
    DLL_EXPORT void InitializeCameraObject(int camID) {
        sensors[camID] = new RGBSensorModule();
    }
    DLL_EXPORT 	void StartCamera(int camID, float fx, float fy, float cx, float cy, float d1, float d2, float d3, float d4) {
         
        if (m_Device != nullptr) {
            Debug::Log("Device is not null");
        }
        else {
            Debug::Log("Device is null");
        }
        sensors[camID]->m_Device = m_Device; 
        sensors[camID]->SetIntrinsicParamters(camID, fx, fy, cx, cy, d1, d2, d3, d4);
        sensorThreads[camID] = new std::thread(startCameraThread, camID);
        sensorThreads[camID]->detach();
    }
    DLL_EXPORT void HookToSensor(int camID) {
        if (sensors.count(camID)) { 
            if (sensors[camID] != nullptr) {
#ifdef __linux
#else   
                sensors[camID]->m_Device = m_Device;
#endif
            } 
        }
    } 
     
    DLL_EXPORT void SetTexturePointer(int camID, void* textureHandle) {
        if (sensors.count(camID)) {
            if (sensors[camID] != nullptr) {
                Debug::Log("Setting Texture Pointer");
                sensors[camID]->SetTexturePointer(textureHandle);
            }
        } 
    }
    DLL_EXPORT void StopCamera(int camID) {
        sensors[camID]->stopFlags = true;
    }
     
    DLL_EXPORT void SubscribeCallbackWithID(int instanceID, int camID, FuncReceiveCameraImageCallbackWithID callback) {
        if (sensors.count(camID)) {
            if (sensors[camID] != nullptr) {
                sensors[camID]->SubscribeReceiver(instanceID, callback);
            } 
        }
    } 
    DLL_EXPORT void SubscribeToInitializedCallback(int camID, FuncTextureInitializedCallback callback) {
        if (sensors.count(camID)) {
            if (sensors[camID] != nullptr) {
                sensors[camID]->textureInitializedCallback = callback;
            }
        }
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
static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType)
{
    // Create graphics API implementation upon initialization
    if (eventType == kUnityGfxDeviceEventInitialize)
    {
#ifdef __linux 
#else  
        IUnityGraphicsD3D11* d3d = s_UnityInterfaces->Get<IUnityGraphicsD3D11>();
        m_Device = d3d->GetDevice(); 
//        Debug::Log("Obtained Device Info");
#endif  
    }
    else if (eventType == kUnityGfxDeviceEventShutdown) {
    }
}
int nextCamToRender = 0;
static void UNITY_INTERFACE_API OnRenderEvent(int eventID)
{
    if (sensors[nextCamToRender] != nullptr) {
        sensors[nextCamToRender]->UpdatecameraTextureGPU();
    }
}
extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc(int camID)
{ 
        nextCamToRender = camID;
        return OnRenderEvent;//just to satisfy unity's needs
}
//Create a callback delegate   
extern "C" DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb) {
    callbackInstance = cb;
}