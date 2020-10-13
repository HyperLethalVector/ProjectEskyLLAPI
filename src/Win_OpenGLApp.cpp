#include "common_header.h"

#include "win_OpenGLApp.h"

COpenGLWinApp appMain;

/*-----------------------------------------------

Name:	ResetTimer

Params:	none

Result:	Resets application timer (for example
		after re-activation of application).

/*---------------------------------------------*/

void COpenGLWinApp::ResetTimer()
{
	dwLastFrame = GetTickCount();
	fFrameInterval = 0.0f;
}

/*-----------------------------------------------

Name:	UpdateTimer

Params:	none

Result:	Updates application timer.

/*---------------------------------------------*/

void COpenGLWinApp::UpdateTimer()
{
	DWORD dwCur = GetTickCount();
	fFrameInterval = float(dwCur-dwLastFrame)*0.001f;
	dwLastFrame = dwCur;
}

/*-----------------------------------------------

Name:	sof

Params:	fVal

Result:	Stands for speed optimized float.

/*---------------------------------------------*/

float COpenGLWinApp::sof(float fVal)
{
	return fVal*fFrameInterval;
}

/*-----------------------------------------------

Name:	InitializeApp

Params:	a_sAppName

Result:	Initializes app with specified (unique)
		application identifier.

/*---------------------------------------------*/

bool COpenGLWinApp::InitializeApp(string a_sAppName)
{
	sAppName = a_sAppName;
	hMutex = CreateMutex(NULL, 1, sAppName.c_str());
//	if(GetLastError() == ERROR_ALREADY_EXISTS)
//	{
//		Debug::Log("The application already is running!");
//		MessageBox(NULL, "This application already runs!", "Multiple Instances Found.", MB_ICONINFORMATION | MB_OK);
//		return 0;
//	}
	doRunLoop = true;
	return 1;
}

/*-----------------------------------------------

Name:	RegisterAppClass

Params:	a_hInstance - application instance handle

Result:	Registers application window class.

/*---------------------------------------------*/

LRESULT CALLBACK GlobalMessageHandler(HWND hWnd, UINT uiMsg, WPARAM wParam, LPARAM lParam)
{
	return appMain.MsgHandlerMain(hWnd, uiMsg, wParam, lParam);
}

void COpenGLWinApp::RegisterAppClass(HINSTANCE a_hInstance)
{
	WNDCLASSEX wcex;
	memset(&wcex, 0, sizeof(WNDCLASSEX));
	wcex.cbSize = sizeof(WNDCLASSEX);
	wcex.style = CS_HREDRAW | CS_VREDRAW | CS_DBLCLKS;

	wcex.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);

	wcex.hIcon = LoadIcon(hInstance, IDI_WINLOGO);
	wcex.hIconSm = LoadIcon(hInstance, IDI_WINLOGO);
	wcex.hCursor = LoadCursor(hInstance, IDC_ARROW);
	wcex.hInstance = hInstance;

	wcex.lpfnWndProc = GlobalMessageHandler;
	wcex.lpszClassName = sAppName.c_str();

	wcex.lpszMenuName = NULL;

	RegisterClassEx(&wcex);
}

/*-----------------------------------------------

Name:	CreateAppWindow

Params:	sTitle - title of created window

Result:	Creates main application window.

/*---------------------------------------------*/

bool COpenGLWinApp::CreateAppWindow(string sTitle)
{
	hWnd = CreateWindowEx(0, sAppName.c_str(), sTitle.c_str(), WS_POPUP,
		x, y, width, height, NULL,
		NULL, hInstance, NULL);
	if (!oglControl.InitOpenGL(hInstance, &hWnd, 3, 1, InitScene, RenderScene, NULL, &oglControl)) {
		return false;
	}
	
	ShowWindow(hWnd, SW_SHOW);

	// Just to send WM_SIZE message again
	ShowWindow(hWnd, SW_SHOWMAXIMIZED);
	UpdateWindow(hWnd);

	return true;
}

/*-----------------------------------------------

Name:	AppBody

Params:	none

Result:	Main application body infinite loop.

/*---------------------------------------------*/

void COpenGLWinApp::AppBody()
{
	MSG msg;
	while(doRunLoop)
	{
		if(PeekMessage(&msg, NULL, 0, 0, PM_REMOVE))
		{
			if(msg.message == WM_QUIT)break; // If the message was WM_QUIT then exit application
			else
			{
				TranslateMessage(&msg); // Otherwise send message to appropriate window
				DispatchMessage(&msg);
			}
		}
		UpdateTimer();
		oglControl.Render(&oglControl);		 
		Sleep(8.33); // Do not consume processor power if application isn't active
	}
}
void COpenGLWinApp::StopInstance() {
	doRunLoop = false;
}
/*-----------------------------------------------

Name:	Shutdown

Params:	none

Result:	Shuts down application and releases used
		memory.

/*---------------------------------------------*/
void COpenGLWinApp::Shutdown()
{
	oglControl.ReleaseOpenGLControl(&oglControl);
	DestroyWindow(hWnd);
	UnregisterClass(sAppName.c_str(), hInstance);
	COpenGLControl::UnregisterSimpleOpenGLClass(hInstance);
	ReleaseMutex(hMutex);
}

/*-----------------------------------------------

Name:	MsgHandlerMain

Params:	windows message stuff

Result:	Application messages handler.

/*---------------------------------------------*/

LRESULT CALLBACK COpenGLWinApp::MsgHandlerMain(HWND hWnd, UINT uiMsg, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;

	switch(uiMsg)
	{
		case WM_PAINT:
			BeginPaint(hWnd, &ps);					
			EndPaint(hWnd, &ps);
			break;

		case WM_CLOSE:
			PostQuitMessage(0);
			break;

		case WM_ACTIVATE:
		{
			switch(LOWORD(wParam))
			{
				case WA_ACTIVE:
				case WA_CLICKACTIVE:
					bAppActive = true;
					break;
				case WA_INACTIVE:
					bAppActive = false;
					break;
			}
			break;
		}

		case WM_SIZE:
			oglControl.ResizeOpenGLViewportFull();
			break;

		default:
			return DefWindowProc(hWnd, uiMsg, wParam, lParam);
	}
	return 0;
}

/*-----------------------------------------------

Name:	GetInstance

Params:	none

Result:	Returns application instance.

/*---------------------------------------------*/

HINSTANCE COpenGLWinApp::GetInstance()
{
	return hInstance;
}