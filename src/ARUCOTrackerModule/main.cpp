#include "ARUCOTrackerModule.h"
#include "common_header.h" 
#define STB_IMAGE_IMPLEMENTATION 
#include "stb_image.h"   
 
#ifdef __linux__
#define DLL_EXPORT  
#else     
#define DLL_EXPORT __declspec(dllexport)    
#endif   
static std::map<int, ARUCOTracker*> trackerModules;
extern "C" { 
	DLL_EXPORT void InitTrackerModule(int dictID, int markerID, float markerLength, int InstanceID) {
		trackerModules[InstanceID] = new ARUCOTracker();
		trackerModules[InstanceID]->instanceID = InstanceID; 
		trackerModules[InstanceID]->InitMarker(dictID, markerID, markerLength);
	} 
	DLL_EXPORT void SubscribeToPoseCallback(int InstanceID, FuncPoseReceiveCallback callback) {
		trackerModules[InstanceID]->subscribeToPoseCallback(callback);
	}
	DLL_EXPORT void ProcessImage(int InstanceID, unsigned char* imageDataRaw, int totalLength, int imageWidth, int imageHeight, int channels) {
		trackerModules[InstanceID]->ProcessImage(imageDataRaw, totalLength, imageWidth, imageHeight, channels);
	}
	DLL_EXPORT void InitARUCOTrackerParams(int InstanceID, float marker_size, float fx, float fy, float cx, float cy, float d1, float d2, float d3, float d4) {
		trackerModules[InstanceID]->InitARUCOTrackerParams(marker_size, fx, fy, cx, cy, d1, d2, d3, d4);
	}
	DLL_EXPORT void PrintMarker(int InstanceID, const char* imageName, int markerID, float markerSize, int borderBits) {
		trackerModules[InstanceID]->PrintMarker(imageName, markerID, markerSize, borderBits);
	} 
	 
    //Create a callback delegate   
    extern "C" DLL_EXPORT void RegisterDebugCallback(FuncCallBack cb) {
        callbackInstance = cb;
    }
}
   