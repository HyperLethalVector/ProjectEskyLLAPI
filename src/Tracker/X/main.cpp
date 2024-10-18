
#include <xv-sdk.h>
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
#include "Tracker.h"   
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

    DLL_EXPORT void ResetFilters(int iD, int d) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->ResetFilters();
    }
    DLL_EXPORT void StopTrackers(int Id) { 
        if (to.find(Id) == to.end()) {return;}
        to[Id]->StopTracking();
    }        
    DLL_EXPORT void UseAsyncHeadPosePredictor(int iD, bool val) {
        if (to.find(iD) == to.end()) { return; }
    } 
    DLL_EXPORT void SetTimeOffset(int Id, float value) {
        if (to.find(Id) == to.end()) { return; }
        to[Id]->UpdateTimeOffset(value);
    }  
    void TrackerBackgroundThread(int i) {     
        if (to.find(i) == to.end()) { return; }
        to[i]->DoFunctionTracking();        
   //     delete to[i];    
 //       to[i] = nullptr;    
    } 
    void PredictorBackgroundThread(int i) { 
        if (to.find(i) == to.end()) { return; }
        return;
    }
    DLL_EXPORT void StartTrackerThread(int Id, bool useLocalization) {//ignored for now.... 
        if (to.find(Id) == to.end()) { return; }
        Debug::Log("Started Tracking Thread");  
        to[Id]->ExitThreadLoop = false;    
        trackerThread[Id] = new std::thread(TrackerBackgroundThread,Id); 
    } 
    DLL_EXPORT float* GetLatestPose(int Id) {      
        if (to.find(Id) == to.end()) { return nullptr; }  
       return to[Id]->latestPose;  
    }   
    DLL_EXPORT float* GetTimestampedPose(int Id) {
        if (to.find(Id) == to.end()) { return nullptr; }
        return to[Id]->latestPose;
    } 

    DLL_EXPORT void InitializeTrackerObject(int Id, int slamMode) {
        Debug::Log("Initializing Tracker Object");
        to[Id] = new TrackerObject();
        to[Id]->TrackerID = Id;
        to[Id]->slamMode = slamMode;
        Debug::Log("Done Initializing Tracker Object");
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
    //Newely Updated Code 

    DLL_EXPORT double* GetLatestTimestampPose(int iD) { 
        if (to.find(iD) == to.end()) { return nullptr; }
        return to[iD]->GetLatestTimestampPose();
    }

    DLL_EXPORT void SetFilterEnabledExt(int iD, bool slam, bool velocity, bool accel, bool angvelocity, bool angaccel) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetFilterEnabledExt(slam, velocity, accel, angvelocity, angaccel);
    } 
    DLL_EXPORT void SetKFilterEnabledExt(int iD, bool slam, bool velocity, bool accel, bool angvelocity, bool angaccel) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetKFilterEnabledExt(slam, velocity, accel, angvelocity, angaccel);
    }
    DLL_EXPORT void UseNewTrackingSystemForParams(int iD) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->trackingSystem = 1;
    }
    DLL_EXPORT void UpdateTransFilterDollaryDooParams(int iD, double _transfreq, double _transmincutoff, double _transbeta, double _transdcutoff,
        double _velfreq, double _velmincutoff, double _velbeta, double _veldcutoff,
        double _accelfreq, double _accelmincutoff, double _accelbeta, double _acceldcutoff
    ) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateTransFilterDollaryDooParams(_transfreq,  _transmincutoff,  _transbeta,  _transdcutoff, 
            _velfreq,  _velmincutoff,  _velbeta,  _veldcutoff, 
            _accelfreq,  _accelmincutoff,  _accelbeta,  _acceldcutoff);
    }
    DLL_EXPORT void UpdateRotFilterDollaryDooParams(int iD, double _rotfreq, double _rotmincutoff, double _rotbeta, double _rotdcutoff,
        double _velfreq, double _velmincutoff, double _velbeta, double _veldcutoff,
        double _accelfreq, double _accelmincutoff, double _accelbeta, double _acceldcutoff
    ) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateRotFilterDollaryDooParams(
            _rotfreq, _rotmincutoff, _rotbeta, _rotdcutoff, 
            _velfreq, _velmincutoff, _velbeta, _veldcutoff, 
            _accelfreq, _accelmincutoff, _accelbeta, _acceldcutoff);
    }

    DLL_EXPORT void UpdateTransFilterKParams(int iD, double _transq, double _transr,
        double _velq, double _velr,
        double _accelq, double _accelr) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateTransFilterKParams(_transq, _transr, _velq, _velr, _accelq, _accelr);
    }
    DLL_EXPORT void UpdateRotFilterKParams(int iD, double _rotq, double _rotr,
        double _angvelq, double _angvelr,
        double _angaccelq, double _angaccelr
    ) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateRotFilterKParams(_rotq, _rotr, _angvelq, _angvelr, _angaccelq, _angaccelr);
    } 

    //old code

     
    DLL_EXPORT void UpdateFilterTranslationParams(int iD, double _freq, double _mincutoff, double _beta, double _dcutoff) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateFilterTranslationParams(_freq, _mincutoff, _beta, _dcutoff); 
    }
    DLL_EXPORT void UpdateFilterRotationParams(int iD, double _freq, double _mincutoff, double _beta, double _dcutoff) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateFilterRotationParams(_freq, _mincutoff, _beta, _dcutoff);
    }  
    DLL_EXPORT void UpdateKFilterTranslationParams(int iD, double _q, double _r) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->UpdateKFilterTranslationParams(_q, _r);
    }
    DLL_EXPORT void UpdateKFilterRotationParams(int iD, double _q, double _r) {
        if (to.find(iD) == to.end()) { return; } 
        to[iD]->UpdateKFilterRotationParams(_q,_r); 
    }
    DLL_EXPORT void SetFilterEnabled(int iD, bool value) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetFilterEnabled(value);  
    }
    DLL_EXPORT void SetKFilterEnabled(int iD, bool value) {
        if (to.find(iD) == to.end()) { return; }
        to[iD]->SetKFilterEnabled(value);
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
        Debug::Log("Registering binary map callback");
        if (to.find(iD) == to.end()) { return; }
        to[iD]->callbackBinaryMap = cb; 
        Debug::Log("Done registering binary map");
    }          
    DLL_EXPORT void SetSerialComPort(int iD, int port) { 
        if (to.find(iD) == to.end()) { return; }
        to[iD]->usesIntegrator = true;   
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
 
          