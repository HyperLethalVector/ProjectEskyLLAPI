
#include <windows.h>

#include "graphics.h"

#include "IUnityInterface.h"
#include "IUnityGraphics.h"

#include <map>

static HWND ghWnd = NULL;
static HWND selectedWnd = NULL;

HINSTANCE hInstance;

static std::map<int, HWND> windowIds;
static std::map<HWND, Graphics> windowGraphics;

void DebugMessage(const char * message);

// this is the main message handler for the program
LRESULT CALLBACK WindowProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    // sort through and find what code to run for the message given
    switch(message)
    {
		case WM_SIZE: // If our window is resizing  
			windowGraphics[hWnd].SetSize(LOWORD(lParam), LOWORD(lParam));
			break; 
			 
		case WM_CLOSE:		
			if(windowGraphics[hWnd].GetCloseFromUnity()){
				windowGraphics[hWnd].SetCloseFromUnity(false);	
				Graphics graphics = windowGraphics[hWnd];
				graphics.GraphicsRelease();
				windowGraphics.erase(hWnd);

			} else {
				return 0;//!! this case, won't be treated, the plugin window should be closed only from Unity ...
			}
			break;	
		

        // this message is read when the window is closed
        case WM_DESTROY:

			break;
    }

    // Handle any messages the switch statement didn't
    return DefWindowProc (hWnd, message, wParam, lParam);
}

HWND CreateNewWindow(LPCWSTR titlestr, int width, int height, bool noStyle) {	
	LPCSTR title = " ";
	if(wcslen(titlestr) > 0){
		title = (LPCSTR)titlestr;		
	}	
	WNDCLASSEXA windowClass;  
	HWND hWnd;  
	ZeroMemory(&windowClass, sizeof(WNDCLASSEX));

	DWORD dwExStyle = WS_EX_APPWINDOW | WS_EX_WINDOWEDGE;  
  
	hInstance = GetModuleHandle(NULL);  

	windowClass.cbSize = sizeof(WNDCLASSEX);
	windowClass.style = CS_HREDRAW | CS_VREDRAW | CS_OWNDC;  
	windowClass.lpfnWndProc = (WNDPROC) WindowProc;  
	windowClass.cbClsExtra = 0;  
	windowClass.cbWndExtra = 0;  
	windowClass.hInstance = hInstance;  
	windowClass.hIcon = LoadIcon(NULL, IDI_WINLOGO);  
	windowClass.hCursor = LoadCursor(NULL, IDC_ARROW);  
	windowClass.hbrBackground = NULL;  
	windowClass.lpszMenuName = NULL;  
	windowClass.lpszClassName = title;
	
	Graphics graphics;
	graphics.SetSize(width, height);

	UnregisterClass((LPCSTR)title, hInstance);

	if (!RegisterClassEx(&windowClass)) {
		return NULL;
	}

	RECT wr = {0, 0, width, height};
	AdjustWindowRect(&wr, WS_OVERLAPPEDWINDOW, FALSE);
	hWnd = CreateWindowEx(dwExStyle, (LPCSTR)title, (LPCSTR)title, WS_OVERLAPPEDWINDOW, CW_USEDEFAULT, 0, wr.right - wr.left, wr.bottom - wr.top, NULL, NULL, hInstance, NULL);
	if(noStyle){
		SetWindowLong(hWnd, GWL_STYLE, 0);
	}

	ShowWindow(hWnd, SW_SHOW);  
	UpdateWindow(hWnd);

	windowGraphics[hWnd] = graphics;

	return hWnd;
} 
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetRenderTextureWidthHeight(int id, int width, int height) {
	if (windowIds.count(id)) {
		selectedWnd = windowIds[id];
		windowGraphics[selectedWnd].unityTextureWidth = width;
		windowGraphics[selectedWnd].unityTextureHeight = height;

	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StartWindow(const wchar_t* title, int width, int height, bool noBorder){
	if(NULL == ghWnd){
		ghWnd = CreateNewWindow(title, width, height, noBorder);
	}
} 

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StopWindow(){
	if(ghWnd){
		windowGraphics[ghWnd].SetCloseFromUnity(true);
		PostMessage(ghWnd, WM_CLOSE, 0, 0); 
		ghWnd = NULL;
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWindowRect(int left, int top, int width, int height) {
	if (ghWnd) {
		windowGraphics[ghWnd].SetSize(width, height);
		SetWindowPos(ghWnd, 0, left, top, width, height, 0);
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPlugin(void* texturePtr){
	selectedWnd = ghWnd;
	windowGraphics[selectedWnd].SetTexturePtrLeft(texturePtr);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginLeft(void* texturePtr) {
	selectedWnd = ghWnd;
	windowGraphics[selectedWnd].SetTexturePtrLeft(texturePtr);
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginRight(void* texturePtr) {
	selectedWnd = ghWnd;
	windowGraphics[selectedWnd].SetTexturePtrRight(texturePtr);
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StartWindowById(int windowId, const wchar_t * title, int width, int height, bool noBorder) {
	if (!windowIds.count(windowId)) {
		windowIds[windowId] = CreateNewWindow(title, width, height, noBorder);
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API StopWindowById(int windowId) {
	if (windowIds.count(windowId)) {
		HWND hwnd = windowIds[windowId];
		windowGraphics[hwnd].SetCloseFromUnity(true);
		windowIds.erase(windowId);
		PostMessage(hwnd, WM_CLOSE, 0, 0);
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWindowRectById(int windowId, int left, int top, int width, int height) {
	if (windowIds.count(windowId)) {
		HWND hwnd = windowIds[windowId];
		windowGraphics[hwnd].SetSize(width, height);
		SetWindowPos(hwnd, 0, left, top, width, height, 0);
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginByIdLeft(int windowId, void* texturePtr) {
	if (windowIds.count(windowId)) {
		selectedWnd = windowIds[windowId];
		windowGraphics[selectedWnd].SetTexturePtrLeft(texturePtr);
	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetRequiredValuesById(int windowID,
float leftUvToRectX[],// = { 0.0 };
float leftUvToRectY[],// = { 0.0 };
float rightUvToRectX[],// = { 0.0 };
float rightUvToRectY[],// = { 0.0 };
float CameraMatrixLeft[],// = { 0.0 };
float CameraMatrixRight[],// = { 0.0 };
float leftOffset[],// = { 0.0 };
float rightOffset[],// = { 0.0 };
float eyeBorders[]) {
	if (windowIds.count(windowID)) {
		selectedWnd = windowIds[windowID];
		windowGraphics[selectedWnd].SetInformation(leftUvToRectX, leftUvToRectY, rightUvToRectX, rightUvToRectY, CameraMatrixLeft, CameraMatrixRight, leftOffset, rightOffset, eyeBorders);
	}
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SendTextureIdToPluginByIdRight(int windowId, void* texturePtr) {
	if (windowIds.count(windowId)) {
		selectedWnd = windowIds[windowId];
		windowGraphics[selectedWnd].SetTexturePtrRight(texturePtr);
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

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWindowAttributes(int colorKey, byte alpha, int flags) {
	if (ghWnd) {
		SetAttributes(ghWnd, colorKey, alpha, flags);
	}
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetWindowAttributesById(int windowId, int colorKey, byte alpha, int flags) {
	if (windowIds.count(windowId)) {
		SetAttributes(windowIds[windowId], colorKey, alpha, flags);
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
	windowGraphics[selectedWnd].InitD3D(selectedWnd);
}

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API InitGraphics(){
	return OnInitGraphics;
}

static void UNITY_INTERFACE_API OnRenderEvent(int eventID) {
	std::map<HWND, Graphics>::iterator it = windowGraphics.begin();
	while (it != windowGraphics.end()) {
		it->second.RenderFrame(); 
		it++;
	}
}

extern "C" UnityRenderingEvent UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API GetRenderEventFunc(){
	return OnRenderEvent;
}

