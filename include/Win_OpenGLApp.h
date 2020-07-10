#pragma once

#include "openGLControl.h"
using namespace std;
class COpenGLWinApp
{
public:
	HWND hWnd; // Handle to application window
	COpenGLControl oglControl; // OpenGL Control
	void StopInstance();
	void ResetTimer();
	void UpdateTimer();
	float sof(float fVal);

	bool InitializeApp(string a_sAppName);
	void RegisterAppClass(HINSTANCE hAppInstance);
	bool CreateAppWindow(string sTitle);
	int x, y, width, height = 0;
	int LeftTexture, RightTexture = 0;
	void AppBody();
	void Shutdown();

	HINSTANCE GetInstance();

	LRESULT CALLBACK MsgHandlerMain(HWND hWnd, UINT uiMsg, WPARAM wParam, LPARAM lParam);

private:
	HINSTANCE hInstance; // Application's instance
	string sAppName;
	HANDLE hMutex;
	bool doRunLoop = true;
	bool bAppActive; // To check if application is active (not minimized)
	DWORD dwLastFrame;
	float fFrameInterval;
};

namespace Keys
{
	int Key(int iKey);
	int Onekey(int iKey);
	extern char kp[256];
}

extern COpenGLWinApp appMain;

void InitScene(LPVOID), RenderScene(LPVOID), ReleaseScene(LPVOID);