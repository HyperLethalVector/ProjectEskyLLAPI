 
#include <windows.h>
#include <thread>
#include "graphics.h"     
#include "IUnityInterface.h" 
#include "IUnityGraphics.h"  
#include <sstream> 
#include <map>    
#include <iostream>

HINSTANCE hInstance;
std::map<int, std::thread*> myThreads;// = nullptr;
std::map<int, HWND> windowIds;
std::map<HWND, int> reverseLookup;//this is here just so we can kill the prior processes 
std::map<int, Graphics*> windowGraphics;

// this is the main message handler for the program
LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{   
    // sort through and find what code to run for the message given 
    switch(message) 
    {
		case WM_SIZE: // If our window is resizing   
			windowGraphics[reverseLookup[hWnd]]->SetSize(LOWORD(lParam), LOWORD(lParam));
			break; 
			 
		case WM_CLOSE:		 
			if (windowGraphics[reverseLookup[hWnd]]->GetCloseFromUnity()) {
				DebugMessage(L"Received Posted Message");
				windowGraphics[reverseLookup[hWnd]]->SetCloseFromUnity(false);
				windowGraphics[reverseLookup[hWnd]]->GraphicsRelease();
				DebugMessage(L"Finished Posted Message, should return to normal thread now");
			}
			else {
				return 0;//!! this case, won't be treated, the plugin window should be closed only from Uni ty ...
			}

			break;	 
		

        // this message is read when the window is closed
        case WM_DESTROY:

			break;  
    } 
	 
    // Handle any messages the switch statement didn't
    return DefWindowProc (hWnd, message, wParam, lParam);
}  

HWND CreateNewWindow(int windowId, int width, int height, bool noStyle) {	
	LPWSTR title = L"EskyNativeRenderer";
	//if(wcslen(titlestr) > 0){
	//	title = (LPCSTR)titlestr;		
//	}	 
	WNDCLASSEXW windowClass;  
	HWND hWnd;  
	DebugMessage(L"Zeroing memory");
	ZeroMemory(&windowClass, sizeof(WNDCLASSEX));

	DWORD dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;  
	DebugMessage(L"Get module handle");
	hInstance = GetModuleHandle(NULL);  

	windowClass.cbSize = sizeof(WNDCLASSEX);
	windowClass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;  
	windowClass.lpfnWndProc = (WNDPROC) WindowProc;  
	windowClass.cbClsExtra = 0;   
	windowClass.cbWndExtra = 0;  
	windowClass.hInstance = hInstance;   
	windowClass.hIcon = LoadIcon(NULL, IDI_WINLOGO);  
	windowClass.hCursor = LoadCursor(NULL, IDC_ARROW);  
	windowClass.hbrBackground = (HBRUSH)COLOR_BACKGROUND;
	windowClass.lpszMenuName = NULL;  
	windowClass.lpszClassName = title;
	
	DebugMessage(L"Unregistering class");
	  
	UnregisterClass(title, hInstance);

	DebugMessage(L"Registering class EX");
	if (!RegisterClassEx(&windowClass)) {  
		return NULL; 
	}  

	RECT wr = {0, 0, width, height};
	DebugMessage(L"Adjusting Window Rect");
	AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
	DebugMessage(L"Create Window EX");
	hWnd = CreateWindowEx(dwExStyle, title, title, WS_OVERLAPPEDWINDOW | WS_VISIBLE, CW_USEDEFAULT, 0, wr.right - wr.left, wr.bottom - wr.top, NULL, NULL, hInstance, NULL);

	if(noStyle){
		DebugMessage(L"No style! set window long");
		SetWindowLong(hWnd, GWL_STYLE, 0);
	} 
	DebugMessage(L"Paint it black~");
	HBRUSH brush = CreateSolidBrush(RGB(0, 0, 0));
	DebugMessage(L"Set the class long pointer");
	SetClassLongPtr(hWnd, GCLP_HBRBACKGROUND, (LONG_PTR)brush); 
	
	DebugMessage(L"Update the window");
	UpdateWindow(hWnd); 
	DebugMessage(L"Creating the graphics class");
	Graphics* graphics = new Graphics();
	DebugMessage(L"Setting the size");
	graphics->SetSize(width, height);
	DebugMessage(L"Storing the graphics class");
	windowGraphics[windowId] = graphics;
	DebugMessage(L"Show the window");
	ShowWindow(hWnd, SW_SHOW);    
	return hWnd; 
} 
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetRenderTextureWidthHeight(int id, int width, int height) {
	if (windowIds.count(id)) { 
		windowGraphics[id]->unityTextureWidth = width;
		windowGraphics[id]->unityTextureHeight = height;
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetOnReceivedFrameCallback(int id, RenderedFrameCallback rfc) {
	if (windowIds.count(id)) {
		windowGraphics[id]->renderedFrameCallback = rfc;
	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetEnableFlagWarping(int id, bool enabled) {
	if (windowIds.count(id)) {
		windowGraphics[id]->SetEnableFlagWarping(enabled);
	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetBrightness(int ID, float brightness) {
	if (windowIds.count(ID)) {
		windowGraphics[ID]->SetBrightness(brightness);
	} 
}  
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StartWindowById(int windowId, int width, int height, bool noBorder) {

	DebugMessage(L"Starting the window");
	windowIds[windowId] = CreateNewWindow(windowId, width, height, noBorder);
	reverseLookup[windowIds[windowId]] = windowId; 
}
void DoBackgroundRender(int ID) { 
	windowGraphics[ID]->hasExitedGraphicsThread = false;
	windowGraphics[ID]->GraphicsBackgroundThreadRenderFrame();
	windowGraphics[ID]->hasExitedGraphicsThread = true;
}   
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StopWindowById(int windowId) {
	DebugMessage(L"Something is closing me");
	windowGraphics[windowId]->SetCloseFromUnity(true);
	myThreads[windowId]->join();
	SendMessage(windowIds[windowId], WM_CLOSE, 0, 0);
	DebugMessage(L"Back to normal thread, should be safe to close the window ID");
	delete myThreads[windowId];
	myThreads.erase(windowId);
	delete windowGraphics[windowId];
	windowGraphics.erase(windowId);
	myThreads.clear();
	windowGraphics.clear();
	reverseLookup.clear();
} 
  
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWindowRectById(int windowId, int left, int top, int width, int height) {
	if (windowIds.count(windowId)) {
		HWND hwnd = windowIds[windowId]; 
		windowGraphics[reverseLookup[hwnd]]->SetSize(width, height);
		SetWindowPos(hwnd, 0, left, top, width, height, 0);   
	}    
} 
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginByIdLeft(int windowId, void* texturePtr) {
	if (windowIds.count(windowId)) { 
		windowGraphics[windowId]->SetTexturePtrLeft(texturePtr);
	}      
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetDeltas(int windowId, void* deltaLeft, void* deltaRight) {
	if (windowIds.count(windowId)) {
		windowGraphics[windowId]->SetAffine((float*)deltaLeft, (float*)deltaRight);
	}       
} 

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetRenderTexturePointerLuT(int iD, void* textureHandleLeftLuT, void* textureHandleRightLuT) {
	if (windowIds.find(iD) == windowIds.end()) { return; }
	windowGraphics[iD]->SetTexturePtrLuTs(textureHandleLeftLuT, textureHandleRightLuT);
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetRequiredValuesById(int windowId,
float leftUvToRectX[],// = { 0.0 };
float leftUvToRectY[],// = { 0.0 };  
float rightUvToRectX[],// = { 0.0 };   
float rightUvToRectY[],// = { 0.0 };
float CameraMatrixLeft[],// = { 0.0 };
float CameraMatrixRight[],// = { 0.0 };
float InvCameraMatrixLeft[],// = { 0.0 };
float InvCameraMatrixRight[],// = { 0.0 };
float leftOffset[],// = { 0.0 };
float rightOffset[],// = { 0.0 };  
float eyeBorders[]) {
	if (windowIds.count(windowId)) {
		windowGraphics[windowId]->SetInformation(leftUvToRectX, leftUvToRectY, rightUvToRectX, rightUvToRectY, CameraMatrixLeft, CameraMatrixRight,InvCameraMatrixLeft,InvCameraMatrixRight, leftOffset, rightOffset, eyeBorders);
	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginByIdRight(int windowId, void* texturePtr) {
	if (windowIds.count(windowId)) {
		windowGraphics[windowId]->SetTexturePtrRight(texturePtr);
	}   
}   
static void SetAttributes(HWND hwnd, int colorKey, byte alpha, int flags) {
	if (flags == 0) {
		SetWindowLong(hwnd, GWL_EXSTYLE, GetWindowLong(hwnd, GWL_EXSTYLE) & ~WS_EX_LAYERED);
	} else {
		SetWindowLong(hwnd, GWL_EXSTYLE, GetWindowLong(hwnd, GWL_EXSTYLE) | WS_EX_LAYERED);
		COLORREF coloref = RGB((colorKey & 0xff0000) >> 16, (colorKey & 0x00ff00) >> 8, colorKey & 0x0000ff);
		if (flags == 1) {
			SetLayeredWindowAttributes(hwnd, coloref, alpha, LWA_COLORKEY);
		} else if (flags == 2) {
			SetLayeredWindowAttributes(hwnd, coloref, alpha, LWA_ALPHA); 
		} else if (flags == 3) {
			SetLayeredWindowAttributes(hwnd, coloref, alpha, LWA_COLORKEY | LWA_ALPHA);
		}
	} 
}  
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetColorFormat(int colorFormat) {
	if (colorFormat == 0) {
		Graphics::colorFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
	} else if (colorFormat == 1) {
		Graphics::colorFormat = DXGI_FORMAT_R16G16B16A16_FLOAT;
	} else if (colorFormat == 2) { 
		Graphics::colorFormat = DXGI_FORMAT_R32G32B32A32_FLOAT;
	}        
} 
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetQualitySettings(int count, int quality) {
	Graphics::sampleCount = count; 
	Graphics::descQuality = quality;   
}   
   
static void UNITY_INTERFACE_API OnInitGraphics(int eventID){ 
	windowGraphics[eventID]->InitD3D(windowIds[eventID]);
	myThreads[eventID] = new std::thread(DoBackgroundRender,eventID);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API InitializeGraphics(int windowID) {
	DebugMessage(L"Initializing Graphics");
	windowGraphics[windowID]->InitD3D(windowIds[windowID]);
//	windowGraphics[windowID].hasExitedGraphicsThread = true;
	myThreads[windowID] = new std::thread(DoBackgroundRender, windowID);
} 

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API InitGraphics(){ 
	return OnInitGraphics;
}

