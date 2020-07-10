#pragma once

#define SIMPLE_OPENGL_CLASS_NAME "Simple_openGL_class"

/********************************

Class:	COpenGLControl

Purpose:	Provides convenient usage
			of OpenGL

********************************/

class COpenGLControl
{
public:
	bool InitOpenGL(HINSTANCE hInstance, HWND* a_hWnd, int iMajorVersion, int iMinorVersion, void (*a_InitScene)(LPVOID), void (*a_RenderScene)(LPVOID), void(*a_ReleaseScene)(LPVOID), LPVOID lpParam);
	
	void ResizeOpenGLViewportFull();

	void Render(LPVOID lpParam);
	void ReleaseOpenGLControl(LPVOID lpParam);

	static void RegisterSimpleOpenGLClass(HINSTANCE hInstance);
	static void UnregisterSimpleOpenGLClass(HINSTANCE hInstance);
	HGLRC unityContext;
	int TextureIDLeft = 0;
	int TextureIDRight = 0;
	void MakeCurrent();
	void SwapBuffersM();
private:
	bool InitGLEW(HINSTANCE hInstance);

	HDC hDC;
	HWND* hWnd;
	HGLRC hRC;
	static bool bClassRegistered;
	static bool bGlewInitialized;
	int iMajorVersion, iMinorVersion;

	void (*InitScene)(LPVOID lpParam), (*RenderScene)(LPVOID lpParam), (*ReleaseScene)(LPVOID lpParam);
};

LRESULT CALLBACK MsgHandlerSimpleOpenGLClass(HWND, UINT, WPARAM, LRESULT);