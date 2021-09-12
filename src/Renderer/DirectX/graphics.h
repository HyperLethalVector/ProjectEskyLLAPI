#pragma once 
#include <thread>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <algorithm>
#include <iterator>
#pragma comment (lib, "d3d11.lib")
#pragma comment (lib, "d3dcompiler.lib")
#include <chrono>
#include "IUnityInterface.h"
#include "IUnityGraphics.h"

#include "IUnityGraphicsD3D11.h"

#include <string>
#include <DirectXMath.h>

struct VERTEX
{
	FLOAT X, Y, Z;      // position
	FLOAT U, V;			//texture 
};
typedef struct Matrix {
	// c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16
}Matrix; 
__declspec(align(16))
typedef struct ShaderVals {//float is 4 bytes
	DirectX::XMFLOAT4X4 leftUvToRectX;
	DirectX::XMFLOAT4X4 leftUvToRectY;
	DirectX::XMFLOAT4X4 rightUvToRectX;
	DirectX::XMFLOAT4X4 rightUvToRectY;
	DirectX::XMFLOAT4X4 cameraMatrixLeft;
	DirectX::XMFLOAT4X4 cameraMatrixRight;
	DirectX::XMFLOAT4X4 InvCameraMatrixLeft;
	DirectX::XMFLOAT4X4 InvCameraMatrixRight;
	DirectX::XMFLOAT4 eyeBordersLeft;
	DirectX::XMFLOAT4 eyeBordersRight;
	DirectX::XMFLOAT4 offsets;
}ShaderVals;
 
typedef struct ShaderVals2 {
	DirectX::XMFLOAT4X4 deltaPoseLeft;
	DirectX::XMFLOAT4X4 deltaPoseRight;
	DirectX::XMFLOAT4 toggleConfigs;
}ShaderVals2;
class WindowContainer {
public:
	HWND myHWND;
	int myID;
};
class Graphics {
private:
	IDXGISwapChain *swapchain;
	ID3D11Device *dev;

	ID3D11RenderTargetView *backbuffer;
	ID3D11VertexShader *pVS;    // the vertex shader
	ID3D11PixelShader *pPS;     // the pixel shader
	ID3D11Buffer *pVBuffer;    // global

	ID3D11InputLayout *pLayout;
	ID3D11SamplerState* pSamplerState; // Texture Sampler


	ID3D11ShaderResourceView* pShaderResourceViewLeft; //Left Eye render texture shader view
	ID3D11ShaderResourceView *pShaderResourceViewRight; //Right Eye render texture shader view

	ID3D11ShaderResourceView* pShaderResourceViewLeftLuT; //Left Eye LuT shader view 
	ID3D11ShaderResourceView* pShaderResourceViewRightLuT; //Right Eye LuT shader view


	ID3D11Texture2D* pTextureLeft; // Shader Texture view left
	ID3D11Texture2D* pTextureRight; // Shader Texture view right
	ID3D11Texture2D* pTextureLeftLuT; // Shader LuT Texture view left
	ID3D11Texture2D* pTextureRightLuT; // Shader LuT Texture view right


	ID3D11Texture2D* pProxyTextureLeft; // Proxy Texture left
	ID3D11Texture2D* pProxyTextureRight; // Proxy Texture right
	ID3D11Texture2D* pProxyTextureLeftLuT; // Proxy LuT  left
	ID3D11Texture2D* pProxyTextureRightLuT; // Proxy LuT  right


	ID3D11Texture2D* pExternalTextureLeft; //Game engine Texture reference (Left)
	ID3D11Texture2D* pExternalTextureRight; // Game engine Texture reference (Right)
	ID3D11Texture2D* pExternalTextureLeftLuT; //Game engine LuT  reference (Left)
	ID3D11Texture2D* pExternalTextureRightLuT; // Game engine LuT  reference (Right)



	ID3D11Buffer*   g_pConstantBuffer11 = NULL;
	ID3D11Buffer* g_pConstantBuffer11_2 = NULL;
	double timeForSwitch = 0.008333;//default is 120fps
	int frameRateSelected = 0; // 0 is 120, 1 is 90, 2 is 60, 3 is 30
	int width;
	int height;
	int bufferWidth = 0;
	int bufferHeight = 0;
	bool wmCloseFromUnity = false;
	//temprorarily stored buffers
	float* bleftUvToRectX;// = { 0.0 };
	float* bleftUvToRectY;// = { 0.0 };
	float* brightUvToRectX;// = { 0.0 };
	float* brightUvToRectY;// = { 0.0 };
	float* bCameraMatrixLeft;// = { 0.0 };
	float* bCameraMatrixRight;// = { 0.0 };
	float* bleftOffset;// = { 0.0 };
	float* brightOffset;// = { 0.0 };
	float* beyeBorders;
	ShaderVals myShaderVals;
	ShaderVals2 myShaderVals2;
	bool updateLuT = false;
public:

	bool threadStarted = false;
	bool hasNewFrame = false;
	bool doesExit = false;
	ID3D11DeviceContext* devcon;
	int unityTextureWidth;
	int unityTextureHeight;
	static DXGI_FORMAT colorFormat;
	static int sampleCount; 
	static int descQuality;
	bool doRender = false;
	bool RenderLock = false;
	bool updateDeltaPoseOnGraphicsThread = false;
	bool graphicsRender = false;
	bool lockRenderingFrame = false;
	void InitD3D(HWND hWnd);
	void GraphicsRelease();
	void RenderFrame();
	void GraphicsBackgroundThreadRenderFrame();
	void SetTexturePtrLeft(void* texturePtr);
	void SetTexturePtrRight(void* texturePtr);
	void SetTexturePtrLuTs(void* texturePtrLeft, void* texturePtrRight);
	void SetSize(int w, int h) { width = w; height = h; }
	void SetCloseFromUnity(bool closeFromUnity) { wmCloseFromUnity = closeFromUnity;}
	bool GetCloseFromUnity() { return wmCloseFromUnity; }
	void SetEnableFlagWarping(bool on) {
		myShaderVals2.toggleConfigs.x = on ? 0.0 : 1.0;
	}
	void SetBrightness(float brightness) {
		myShaderVals2.toggleConfigs.y = brightness;
	} 
	void StartRenderThread() {
		if (!threadStarted) {
			threadStarted = true;
		}
	}
	void StopRenderThread() {
		if (threadStarted) {

		}
	} 
	void SetFrameRate(int FrameRateSelected) {
		frameRateSelected = FrameRateSelected;

		switch (frameRateSelected) {
			case 0:
				timeForSwitch = 0.00833333333;//120fps
				break;
			case 1:
				timeForSwitch = 0.011f;//90fps
				break;
			case 2:
				timeForSwitch = 0.01666f;//60fps
				break;
			case 3:
				timeForSwitch = 0.03333f;//30fps

		}
	}
	void SetAffine(float* inputDeltaLeft, float* inputDeltaRight) {
		if (!RenderLock) {
			RenderLock = true;
			for (int x = 0; x < 4; x++) {
				for (int y = 0; y < 4; y++) {
					myShaderVals2.deltaPoseLeft.m[x][y] = inputDeltaLeft[y * 4 + x];
					myShaderVals2.deltaPoseRight.m[x][y] = inputDeltaRight[y * 4 + x];
				}
			}
			updateDeltaPoseOnGraphicsThread = true;
			RenderLock = false;
		}
	}
	void SetInformation(float leftUvToRectX[],// = { 0.0 };
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
		  
		myShaderVals.eyeBordersLeft.x = eyeBorders[0];myShaderVals.eyeBordersLeft.y = eyeBorders[1];myShaderVals.eyeBordersLeft.z = eyeBorders[2];myShaderVals.eyeBordersLeft.w = eyeBorders[3];
		myShaderVals.eyeBordersRight.x = eyeBorders[4];myShaderVals.eyeBordersRight.y = eyeBorders[5];myShaderVals.eyeBordersRight.z = eyeBorders[6];myShaderVals.eyeBordersRight.w = eyeBorders[7];
		myShaderVals.offsets.x = leftOffset[0]; myShaderVals.offsets.y = leftOffset[1]; myShaderVals.offsets.z = rightOffset[0]; myShaderVals.offsets.w = rightOffset[1];
		for (int x = 0; x < 4; x++) {
			for (int y = 0; y < 4; y++) {
				myShaderVals.leftUvToRectX.m[x][y] = leftUvToRectX[y * 4 + x];
				myShaderVals.leftUvToRectY.m[x][y] = leftUvToRectY[y * 4 + x];
				myShaderVals.rightUvToRectX.m[x][y] = rightUvToRectX[y * 4 + x];
				myShaderVals.rightUvToRectY.m[x][y] = rightUvToRectY[y * 4 + x];
				myShaderVals.cameraMatrixLeft.m[x][y] = CameraMatrixLeft[y * 4 + x];
				myShaderVals.cameraMatrixRight.m[x][y] = CameraMatrixRight[y * 4 + x];
				myShaderVals.InvCameraMatrixLeft.m[x][y] = InvCameraMatrixLeft[y * 4 + x];
				myShaderVals.InvCameraMatrixRight.m[x][y] = InvCameraMatrixRight[y * 4 + x];
				myShaderVals2.deltaPoseLeft.m[x][y] = 0.0f;
				myShaderVals2.deltaPoseRight.m[x][y] = 0.0f;
			}
		}
		for (int x = 0; x < 4; x++) {
			myShaderVals2.deltaPoseLeft.m[x][x] = 1.0;
			myShaderVals2.deltaPoseRight.m[x][x] = 1.0;
		}
		if (g_pConstantBuffer11) {
			D3D11_MAPPED_SUBRESOURCE mappedResource;
			devcon->Map(g_pConstantBuffer11, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
			ShaderVals* dataPtr = (ShaderVals*)mappedResource.pData;
			dataPtr->leftUvToRectX = myShaderVals.leftUvToRectX;
			dataPtr->leftUvToRectY = myShaderVals.leftUvToRectY;
			dataPtr->rightUvToRectX = myShaderVals.rightUvToRectX;
			dataPtr->rightUvToRectY = myShaderVals.rightUvToRectY;
			dataPtr->cameraMatrixLeft = myShaderVals.cameraMatrixLeft;
			dataPtr->cameraMatrixRight = myShaderVals.cameraMatrixRight;
			dataPtr->eyeBordersLeft = myShaderVals.eyeBordersLeft;
			dataPtr->eyeBordersRight = myShaderVals.eyeBordersRight;
			dataPtr->InvCameraMatrixLeft = myShaderVals.InvCameraMatrixLeft;
			dataPtr->InvCameraMatrixRight = myShaderVals.InvCameraMatrixRight;
			dataPtr->offsets = myShaderVals.offsets;
			devcon->Unmap(g_pConstantBuffer11, 0);
			devcon->VSSetConstantBuffers(0, 1, &g_pConstantBuffer11);
			devcon->PSSetConstantBuffers(0, 1, &g_pConstantBuffer11);
		}
		if (g_pConstantBuffer11_2) {
			D3D11_MAPPED_SUBRESOURCE mappedResource;
			devcon->Map(g_pConstantBuffer11_2, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
			ShaderVals2* dataPtr = (ShaderVals2*)mappedResource.pData;
			dataPtr->deltaPoseLeft = myShaderVals2.deltaPoseLeft;
			dataPtr->deltaPoseRight = myShaderVals2.deltaPoseRight;
			devcon->Unmap(g_pConstantBuffer11_2, 0);
			devcon->VSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2);
			devcon->PSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2); 
		}
	}
private:
	void SetViewport(int width, int height);
	void SetBufferRenderTargets();
	void InitPipeline(); // loads and prepares the shaders
	void CreateProxyTextureLeft();
	void CreateProxyTextureRight();
	void CreateProxyLuT();
	void UseExternalTexture();
	void InitTextureSampler();
	void InitGraphics();// creates the shape to render
	void CleanD3D();
};


