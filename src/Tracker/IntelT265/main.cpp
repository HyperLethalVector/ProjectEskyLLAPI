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
#ifdef __linux__
#else 
#include <d3d11.h>  
#define STB_IMAGE_IMPLEMENTATION
#endif 
//-------------------------------------------------------------------
  
#ifdef __linux__ 
#define DLL_EXPORT 
#else  
#define DLL_EXPORT __declspec(dllexport) 
#endif  
#ifdef __cplusplus     
extern "C" {   
#endif      
    map<int,TrackerObject*> to;
    map<int,std::thread*> t3;     
#ifdef __linux  
#else    
    ID3D11Device* m_Device;  
#endif  
    DLL_EXPORT void StopTrackers(int Id) { 
        to[Id]->ExitThreadLoop = true;
        to[Id]->StopTracking(); 
    }           
    void TrackerBackgroundThread(int i) {     
        to[i]->DoFunctionTracking();    
        delete to[i];   
        to[i] = nullptr;   
    }
    DLL_EXPORT void StartTrackerThread(int Id, bool useLocalization) {//ignored for now....
        Debug::Log("Started Tracking Thread");  
        to[Id]->ExitThreadLoop = false;   
        t3[Id] = new std::thread(TrackerBackgroundThread,Id);
    } 
    DLL_EXPORT float* GetLatestPose(int Id) {      
       return to[Id]->latestPose;  
    }   
    DLL_EXPORT void InitializeTrackerObject(int Id) {
        to[Id] = new TrackerObject(); 
        to[Id]->TrackerID = Id; 
    }
    DLL_EXPORT void ObtainMap(int Id) { 
        if (to.count(Id)) {
            to[Id]->GrabMap();
        }
    } 
    DLL_EXPORT void FlagMapImport(int Id) {
       to[Id]->FlagMapImport();
    }  
    DLL_EXPORT void ObtainOriginInLocalizedMap(int Id) {
            to[Id]->GrabPoseInOrigin();
    }
    DLL_EXPORT void HookDeviceToIntel(int Id) {
#ifdef __linux
#else
            to[Id]->m_Device = m_Device;
#endif
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
    DLL_EXPORT void SetTextureInitializedCallback(int iD, FuncTextureInitializedCallback myCallback) {
        to[iD]->textureInitializedCallback = myCallback;    
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
#endif         
        }            
        else if (eventType == kUnityGfxDeviceEventShutdown) {  
        }               
    }              
    DLL_EXPORT void RegisterMatrixDeltaConvCallback(int iD, FuncDeltaMatrixConvertCallback callback) {
       to[iD]->callbackMatrixConvert = callback;
    } 
    DLL_EXPORT void RegisterQuaternionConversionCallback(int iD, QuaternionCallback qc) {  
        to[iD]->quaternionCallback = qc;    
    } 
    DLL_EXPORT void RegisterDeltaPoseUpdate(int iD, FuncDeltaPoseUpdateCallback fdpuc) {
        to[iD]->callbackDeltaPoseUpdate = fdpuc; 
    }      
    DLL_EXPORT void PostRenderReset(int iD) {     
        to[iD]->ResetInitialPose();  
    } 
    DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb) {
        callbackInstance = cb;
    }
    DLL_EXPORT void RegisterLocalizationCallback(int iD, LocalizationCallback cb) {
        to[iD]->callbackLocalization = cb;
    }
    DLL_EXPORT void RegisterObjectPoseCallback(int iD, LocalizationPoseCallback cb) {
        to[iD]->callbackObjectPoseReceived = cb;
    }
    DLL_EXPORT void RegisterBinaryMapCallback(int iD, MapDataCallback cb) {
        to[iD]->callbackBinaryMap = cb; 
    }   
    DLL_EXPORT void SetSerialComPort(int iD, int port) { 
        to[iD]->usesIntegrator = true; 
        to[iD]->SetComPortString(port); 
    }      
    DLL_EXPORT void SetRenderTexturePointer(int iD, void* textureHandle) {
        to[iD]->SetTexturePointer(textureHandle);
    } 
    DLL_EXPORT void SubscribeCallbackImageWithID(int iD, int instanceID, FuncReceiveCameraImageCallbackWithID callback) {
        to[iD]->SubscribeReceiver(callback,instanceID);
    }           
    DLL_EXPORT void SetLeftRightEyeTransform(int iD, float* leftEyeTransform, float* rightEyeTransform) {
        to[iD]->SetLeftRightEyeTransforms(leftEyeTransform, rightEyeTransform);
    }
}        
 
static void UNITY_INTERFACE_API OnRenderEvent(int iD)
{ 
   to[iD]->UpdatecameraTextureGPU(); 
}
extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc(int iD)
{ 
    return OnRenderEvent;
}     
 
 
//Create a callback delegate   
 
         