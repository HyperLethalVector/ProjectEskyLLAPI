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
    map<int,std::thread*> trackerThread;     
    map<int, std::thread*> asyncPredictor;
#ifdef __linux       
#else    
    ID3D11Device* m_Device;  
#endif     
    DLL_EXPORT void StopTrackers(int Id) {  
        if (to.find(Id) == to.end()) {return;}
        to[Id]->ExitThreadLoop = true;
        to[Id]->StopTracking();  
        while (!to[Id]->hasExitedPredictorThread || !to[Id]->hasExitedPredictorThread) {
            //Infinite loop if it's not exited
            Debug::Log("Stopping the tracker");
        }
    }           
    DLL_EXPORT void SetTimeOffset(int Id, float value) {
        if (to.find(Id) == to.end()) { return; }
        to[Id]->UpdateTimeOffset(value);
    }
    void TrackerBackgroundThread(int i) {     
        if (to.find(i) == to.end()) { return; }
        to[i]->DoFunctionTracking();    
        to[i]->hasExitedPredictorThread = true;
   //     delete to[i];   
 //       to[i] = nullptr;   
    }
    void PredictorBackgroundThread(int i) {
        if (to.find(i) == to.end()) { return; }
        to[i]->FunctionHeadPosePredictor();  
        to[i]->hasExitedTrackerThread = true;
    }
    DLL_EXPORT void StartTrackerThread(int Id, bool useLocalization) {//ignored for now.... 
        if (to.find(Id) == to.end()) { return; }
        Debug::Log("Started Tracking Thread");  
        to[Id]->ExitThreadLoop = false;    
        to[Id]->hasExitedTrackerThread = false;
        to[Id]->hasExitedPredictorThread = false;
        trackerThread[Id] = new std::thread(TrackerBackgroundThread,Id); 
        asyncPredictor[Id] = new std::thread(PredictorBackgroundThread, Id);
    } 
    DLL_EXPORT float* GetLatestPose(int Id) {      
        if (to.find(Id) == to.end()) { return nullptr; } 
       return to[Id]->latestPose;  
    }   
    DLL_EXPORT void InitializeTrackerObject(int Id) {
        to[Id] = new TrackerObject(); 
        to[Id]->TrackerID = Id; 
    }
    DLL_EXPORT void ObtainMap(int Id) { 
        if (to.find(Id) == to.end()) { return; } 
        if (to.count(Id)) {
            to[Id]->GrabMap();
        }
    } 
    DLL_EXPORT void FlagMapImport(int Id) {
        if (to.find(Id) == to.end()) { return; }
       to[Id]->FlagMapImport();
    }  
    DLL_EXPORT void ObtainOriginInLocalizedMap(int Id) {
        if (to.find(Id) == to.end()) { return; }
            to[Id]->GrabPoseInOrigin();
    }
    DLL_EXPORT void HookDeviceToIntel(int Id) { 
#ifdef __linux
#else
        if (to.find(Id) == to.end()) { return; }
            to[Id]->m_Device = m_Device;
#endif
    }
    static IUnityInterfaces* s_UnityInterfaces = NULL;  
    static IUnityGraphics* s_Graphics = NULL;
     
    static void UNITY_INTERFACE_API OnGraphicsDeviceEvent(UnityGfxDeviceEventType eventType); 
    extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces * unityInterfaces)
    {
        s_UnityInterfaces = unityInterfaces;  
        if (s_UnityInterfaces != nullptr) {
            s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
            if (s_Graphics != nullptr) {
                s_Graphics->RegisterDeviceEventCallback(OnGraphicsDeviceEvent);
                OnGraphicsDeviceEvent(kUnityGfxDeviceEventInitialize); 
            }  
        }
    }   
       
    extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload() 
    { 
        if (s_Graphics != nullptr) {
            s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
        }
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
            if (d3d != nullptr) { 
                m_Device = d3d->GetDevice();
            }
#endif          
        }             
        else if (eventType == kUnityGfxDeviceEventShutdown) {  
        }                
    }               
    DLL_EXPORT void UpdateFilterTranslationParams(int iD, double _freq, double _mincutoff, double _beta, double _dcutoff) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateFilterTranslationParams(_freq, _mincutoff, _beta, _dcutoff); 
    }
    DLL_EXPORT void UpdateFilterRotationParams(int iD, double _freq, double _mincutoff, double _beta, double _dcutoff) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateFilterRotationParams(_freq, _mincutoff, _beta, _dcutoff);
    }  
    DLL_EXPORT void SetFilterEnabled(int iD, bool value) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetFilterEnabled(value);  
    }
    DLL_EXPORT void RegisterMatrixDeltaConvCallback(int iD, FuncDeltaMatrixConvertCallback callback) {
        if (to.find(iD) == to.end()) { return; }
       to[iD]->callbackMatrixConvert = callback; 
    }  
    DLL_EXPORT void RegisterDeltaPoseUpdate(int iD, FuncDeltaPoseUpdateCallback fdpuc) {
        if (to.find(iD) == to.end()) { return; } 
        to[iD]->callbackDeltaPoseUpdate = fdpuc;  
    }      
    DLL_EXPORT void EnablePassthrough(int iD, bool enabled) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->initializeWithPassthrough = enabled;
    }
    DLL_EXPORT void PostRenderReset(int iD) {   
        if (to.find(iD) == to.end()) { return; } 
        to[iD]->ResetInitialPose();   
    } 
    DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb) {  
        callbackInstance = cb;
    }   
    DLL_EXPORT void RegisterLocalizationCallback(int iD, LocalizationCallback cb) {
        if (to.find(iD) == to.end()) { return; } 
        to[iD]->callbackLocalization = cb; 
    }  
    DLL_EXPORT void RegisterObjectPoseCallback(int iD, LocalizationPoseCallback cb) { 
        if (to.find(iD) == to.end()) { return; }
        to[iD]->callbackObjectPoseReceived = cb;  
    }   
    DLL_EXPORT void RegisterBinaryMapCallback(int iD, MapDataCallback cb) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->callbackBinaryMap = cb; 
    }           
    DLL_EXPORT void SetSerialComPort(int iD, int port) {  
        if (to.find(iD) == to.end()) { return; }
        to[iD]->usesIntegrator = true;  
        to[iD]->SetComPortString(port); 
    }        
    DLL_EXPORT void SetRenderTexturePointer(int iD, void* textureHandle) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetTexturePointer(textureHandle);  
    }   

    DLL_EXPORT void SubscribeCallbackImageWithID(int iD, int instanceID, FuncReceiveCameraImageCallbackWithID callback) {
        to[iD]->SubscribeReceiver(callback,instanceID);
    }           
    DLL_EXPORT void SetLeftRightEyeTransform(int iD, float* leftEyeTransform, float* rightEyeTransform) {
        if (to.find(iD) == to.end()) { return; }
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
 
         