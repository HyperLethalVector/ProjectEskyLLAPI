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
	bool shouldUpdateInternalLinkage = false;

	int TextureIDLeft = 0;
	int TextureIDRight = 0;

	float* CameraMatrixLeft;
	float* CameraMatrixRight;
	float* LeftEyeBorderConstraints;
	float* RightEyeBorderConstraints;
	bool useCameraMatricies = false;
	bool updateOffset_x_y = false;
	bool updateBorders = false;
	//the below is just my defaults
	float* left_offset_x_y = new float[2]{ 0.0,0.0 };
	float* right_offset_x_y = new float[2]{ 0.0,0.0 };
	float* left_uv_to_rect_x = new float[16]{ -0.7530364531010308, 0.8806592947908687, -0.8357813137161849, 0.3013989721607643, 0.9991764544369446, -0.2578159567698274, 0.3278667335649757, -0.4602577277109663, -0.23980700925448195, -0.056891370605734376, -0.1248008903440144, 0.7641381600051023, 0.20935445281014292, -0.06256983016261788, 0.25844580123833516, -0.5098143951663658 };
	float* left_uv_to_rect_y = new float[16]{ 0.5612597403791647, -1.1899589356849427, 0.4652815794139322, -0.2737933233160801, 0.3774305703820774, -0.8110333901413378, 1.2705775357104372, -0.7461290557575936, -0.19222925521894155, 0.936404121235537, -1.7109388784623627, 0.9147182510080394, 0.33073407860855586, -1.1463700238163494, 1.4965795269835196, -0.7090919632511286 };
	float* right_uv_to_rect_x = new float[16]{ -0.2117125319456463, -0.432262579698108, 0.41675063901331316, -0.14650788483832153, 1.0941580384494245, -0.30628109185189906, 0.109119134429531, 0.11642874201014344, -0.2761527408488216, -0.4335709010559027, 0.9626491769528918, -0.5572405188216735, 0.18342869894719088, 0.37981945016058366, -0.8718621504058989, 0.5218968716935535 };
	float* right_uv_to_rect_y = new float[16]{ 1.0129568069314265, -2.110976542118192, 1.4108474581893895, -0.7746290913232183, -0.746419837008027, 1.747642287758405, -1.5753294007072252, 0.7143402603200871, 0.5607717274125551, -1.5019493985594772, 1.2539128525783017, -0.42999735712430215, -0.21517910830152714, 0.5965062719847273, -0.5664205050494074, 0.18545738302854597 };
	void MakeCurrent();
	void SwapBuffersM();
	HDC hDC;
	HGLRC hRC;
private:
	bool InitGLEW(HINSTANCE hInstance);


	HWND* hWnd;

	static bool bClassRegistered;
	static bool bGlewInitialized;
	int iMajorVersion, iMinorVersion;

	void (*InitScene)(LPVOID lpParam), (*RenderScene)(LPVOID lpParam), (*ReleaseScene)(LPVOID lpParam);
};

LRESULT CALLBACK MsgHandlerSimpleOpenGLClass(HWND, UINT, WPARAM, LRESULT);