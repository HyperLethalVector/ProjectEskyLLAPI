#pragma once 
#include <thread>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <algorithm>
#include <iterator>
#pragma comment (lib, "d3d11.lib")
#pragma comment (lib, "d3dcompiler.lib")

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
	DirectX::XMFLOAT4X4 delta;
	DirectX::XMFLOAT4X4 deltaInverse;
	DirectX::XMFLOAT4X4 eyeOffsetPoseLeft;
	DirectX::XMFLOAT4X4 eyeOffsetPoseRight;
	DirectX::XMFLOAT4X4 eyeOffsetPoseLeftInv;
	DirectX::XMFLOAT4X4 eyeOffsetPoseRightInv;
}ShaderVals2;
class Graphics {
private:
	IDXGISwapChain *swapchain;
	ID3D11Device *dev;

	ID3D11RenderTargetView *backbuffer;
	ID3D11VertexShader *pVS;    // the vertex shader
	ID3D11PixelShader *pPS;     // the pixel shader
	ID3D11Buffer *pVBuffer;    // global
	ID3D11InputLayout *pLayout;
	ID3D11ShaderResourceView *pShaderResourceViewLeft;
	ID3D11ShaderResourceView *pShaderResourceViewRight;
	ID3D11SamplerState *pSamplerState;
	ID3D11Texture2D *pTextureLeft;
	ID3D11Texture2D *pTextureRight;
	ID3D11Texture2D *pProxyTextureLeft;
	ID3D11Texture2D *pProxyTextureRight;
	ID3D11Texture2D *pExternalTextureLeft;
	ID3D11Texture2D *pExternalTextureRight;
	ID3D11Buffer*   g_pConstantBuffer11 = NULL;
	ID3D11Buffer* g_pConstantBuffer11_2 = NULL;
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
	bool updateAffineOnGraphicsThread = false;
	bool graphicsRender = false;
	void InitD3D(HWND hWnd);
	void GraphicsRelease();
	void RenderFrame();
	void GraphicsBackgroundThreadRenderFrame();
	void SetTexturePtrLeft(void* texturePtr);
	void SetTexturePtrRight(void* texturePtr);
	void SetSize(int w, int h) { width = w; height = h; }
	void SetCloseFromUnity(bool closeFromUnity) { wmCloseFromUnity = closeFromUnity;}
	bool GetCloseFromUnity() { return wmCloseFromUnity; }
	void StartRenderThread() {
		if (!threadStarted) {
			threadStarted = true;
		}
	}
	void StopRenderThread() {
		if (threadStarted) {

		}
	} 
	void SetEyeOffset(float* inputEyeLeft,float* inputEyeRight, float* inputEyeLeftInv, float* inputEyeRightInv) {
		myShaderVals2.eyeOffsetPoseLeft.m[0][0] = inputEyeLeft[0];
		myShaderVals2.eyeOffsetPoseLeft.m[1][0] = inputEyeLeft[1]; 
		myShaderVals2.eyeOffsetPoseLeft.m[2][0] = inputEyeLeft[2];
		myShaderVals2.eyeOffsetPoseLeft.m[3][0] = inputEyeLeft[3];

		myShaderVals2.eyeOffsetPoseLeft.m[0][1] = inputEyeLeft[4];
		myShaderVals2.eyeOffsetPoseLeft.m[1][1] = inputEyeLeft[5];
		myShaderVals2.eyeOffsetPoseLeft.m[2][1] = inputEyeLeft[6];
		myShaderVals2.eyeOffsetPoseLeft.m[3][1] = inputEyeLeft[7];

		myShaderVals2.eyeOffsetPoseLeft.m[0][2] = inputEyeLeft[8];
		myShaderVals2.eyeOffsetPoseLeft.m[1][2] = inputEyeLeft[9];
		myShaderVals2.eyeOffsetPoseLeft.m[2][2] = inputEyeLeft[10];
		myShaderVals2.eyeOffsetPoseLeft.m[3][2] = inputEyeLeft[11];

		myShaderVals2.eyeOffsetPoseLeft.m[0][3] = inputEyeLeft[12];
		myShaderVals2.eyeOffsetPoseLeft.m[1][3] = inputEyeLeft[13];
		myShaderVals2.eyeOffsetPoseLeft.m[2][3] = inputEyeLeft[14];
		myShaderVals2.eyeOffsetPoseLeft.m[3][3] = inputEyeLeft[15];

		myShaderVals2.eyeOffsetPoseRight.m[0][0] = inputEyeRight[0];
		myShaderVals2.eyeOffsetPoseRight.m[1][0] = inputEyeRight[1];
		myShaderVals2.eyeOffsetPoseRight.m[2][0] = inputEyeRight[2];
		myShaderVals2.eyeOffsetPoseRight.m[3][0] = inputEyeRight[3];

		myShaderVals2.eyeOffsetPoseRight.m[0][1] = inputEyeRight[4];
		myShaderVals2.eyeOffsetPoseRight.m[1][1] = inputEyeRight[5];
		myShaderVals2.eyeOffsetPoseRight.m[2][1] = inputEyeRight[6];
		myShaderVals2.eyeOffsetPoseRight.m[3][1] = inputEyeRight[7];

		myShaderVals2.eyeOffsetPoseRight.m[0][2] = inputEyeRight[8];
		myShaderVals2.eyeOffsetPoseRight.m[1][2] = inputEyeRight[9];
		myShaderVals2.eyeOffsetPoseRight.m[2][2] = inputEyeRight[10];
		myShaderVals2.eyeOffsetPoseRight.m[3][2] = inputEyeRight[11];

		myShaderVals2.eyeOffsetPoseRight.m[0][3] = inputEyeRight[12];
		myShaderVals2.eyeOffsetPoseRight.m[1][3] = inputEyeRight[13];
		myShaderVals2.eyeOffsetPoseRight.m[2][3] = inputEyeRight[14];
		myShaderVals2.eyeOffsetPoseRight.m[3][3] = inputEyeRight[15];
		//Inverse Matricies

		myShaderVals2.eyeOffsetPoseLeftInv.m[0][0] = inputEyeLeftInv[0];
		myShaderVals2.eyeOffsetPoseLeftInv.m[1][0] = inputEyeLeftInv[1];
		myShaderVals2.eyeOffsetPoseLeftInv.m[2][0] = inputEyeLeftInv[2];
		myShaderVals2.eyeOffsetPoseLeftInv.m[3][0] = inputEyeLeftInv[3];

		myShaderVals2.eyeOffsetPoseLeftInv.m[0][1] = inputEyeLeftInv[4];
		myShaderVals2.eyeOffsetPoseLeftInv.m[1][1] = inputEyeLeftInv[5];
		myShaderVals2.eyeOffsetPoseLeftInv.m[2][1] = inputEyeLeftInv[6];
		myShaderVals2.eyeOffsetPoseLeftInv.m[3][1] = inputEyeLeftInv[7];

		myShaderVals2.eyeOffsetPoseLeftInv.m[0][2] = inputEyeLeftInv[8];
		myShaderVals2.eyeOffsetPoseLeftInv.m[1][2] = inputEyeLeftInv[9];
		myShaderVals2.eyeOffsetPoseLeftInv.m[2][2] = inputEyeLeftInv[10];
		myShaderVals2.eyeOffsetPoseLeftInv.m[3][2] = inputEyeLeftInv[11];

		myShaderVals2.eyeOffsetPoseLeftInv.m[0][3] = inputEyeLeftInv[12];
		myShaderVals2.eyeOffsetPoseLeftInv.m[1][3] = inputEyeLeftInv[13];
		myShaderVals2.eyeOffsetPoseLeftInv.m[2][3] = inputEyeLeftInv[14];
		myShaderVals2.eyeOffsetPoseLeftInv.m[3][3] = inputEyeLeftInv[15];

		myShaderVals2.eyeOffsetPoseRightInv.m[0][0] = inputEyeRightInv[0];
		myShaderVals2.eyeOffsetPoseRightInv.m[1][0] = inputEyeRightInv[1];
		myShaderVals2.eyeOffsetPoseRightInv.m[2][0] = inputEyeRightInv[2];
		myShaderVals2.eyeOffsetPoseRightInv.m[3][0] = inputEyeRightInv[3];

		myShaderVals2.eyeOffsetPoseRightInv.m[0][1] = inputEyeRightInv[4];
		myShaderVals2.eyeOffsetPoseRightInv.m[1][1] = inputEyeRightInv[5];
		myShaderVals2.eyeOffsetPoseRightInv.m[2][1] = inputEyeRightInv[6];
		myShaderVals2.eyeOffsetPoseRightInv.m[3][1] = inputEyeRightInv[7];

		myShaderVals2.eyeOffsetPoseRightInv.m[0][2] = inputEyeRightInv[8];
		myShaderVals2.eyeOffsetPoseRightInv.m[1][2] = inputEyeRightInv[9];
		myShaderVals2.eyeOffsetPoseRightInv.m[2][2] = inputEyeRightInv[10];
		myShaderVals2.eyeOffsetPoseRightInv.m[3][2] = inputEyeRightInv[11];

		myShaderVals2.eyeOffsetPoseRightInv.m[0][3] = inputEyeRightInv[12];
		myShaderVals2.eyeOffsetPoseRightInv.m[1][3] = inputEyeRightInv[13];
		myShaderVals2.eyeOffsetPoseRightInv.m[2][3] = inputEyeRightInv[14];
		myShaderVals2.eyeOffsetPoseRightInv.m[3][3] = inputEyeRightInv[15];


		updateAffineOnGraphicsThread = true;
	}
	void SetAffine(float* inputDelta, float* inputdeltaInverse) {
		myShaderVals2.delta.m[0][0] = inputDelta[0];
		myShaderVals2.delta.m[1][0] = inputDelta[1];
		myShaderVals2.delta.m[2][0] = inputDelta[2];
		myShaderVals2.delta.m[3][0] = inputDelta[3];


		myShaderVals2.delta.m[0][1] = inputDelta[4];
		myShaderVals2.delta.m[1][1] = inputDelta[5];
		myShaderVals2.delta.m[2][1] = inputDelta[6];
		myShaderVals2.delta.m[3][1] = inputDelta[7];

		myShaderVals2.delta.m[0][2] = inputDelta[8];
		myShaderVals2.delta.m[1][2] = inputDelta[9];
		myShaderVals2.delta.m[2][2] = inputDelta[10];
		myShaderVals2.delta.m[3][2] = inputDelta[11];

		myShaderVals2.delta.m[0][3] = inputDelta[12];
		myShaderVals2.delta.m[1][3] = inputDelta[13];
		myShaderVals2.delta.m[2][3] = inputDelta[14];
		myShaderVals2.delta.m[3][3] = inputDelta[15];


		myShaderVals2.deltaInverse.m[0][0] = inputdeltaInverse[0];
		myShaderVals2.deltaInverse.m[1][0] = inputdeltaInverse[1];
		myShaderVals2.deltaInverse.m[2][0] = inputdeltaInverse[2];
		myShaderVals2.deltaInverse.m[3][0] = inputdeltaInverse[3];


		myShaderVals2.deltaInverse.m[0][1] = inputdeltaInverse[4];
		myShaderVals2.deltaInverse.m[1][1] = inputdeltaInverse[5];
		myShaderVals2.deltaInverse.m[2][1] = inputdeltaInverse[6];
		myShaderVals2.deltaInverse.m[3][1] = inputdeltaInverse[7];

		myShaderVals2.deltaInverse.m[0][2] = inputdeltaInverse[8];
		myShaderVals2.deltaInverse.m[1][2] = inputdeltaInverse[9];
		myShaderVals2.deltaInverse.m[2][2] = inputdeltaInverse[10];
		myShaderVals2.deltaInverse.m[3][2] = inputdeltaInverse[11];

		myShaderVals2.deltaInverse.m[0][3] = inputdeltaInverse[12];
		myShaderVals2.deltaInverse.m[1][3] = inputdeltaInverse[13];
		myShaderVals2.deltaInverse.m[2][3] = inputdeltaInverse[14];
		myShaderVals2.deltaInverse.m[3][3] = inputdeltaInverse[15];

		updateAffineOnGraphicsThread = true;
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
			}
		}
		myShaderVals2.delta.m[0][0] = 1.0;
		myShaderVals2.delta.m[1][0] = 0.0;
		myShaderVals2.delta.m[2][0] = 0.0;
		myShaderVals2.delta.m[0][1] = 0.0; 
		myShaderVals2.delta.m[1][1] = 1.0;
		myShaderVals2.delta.m[2][1] = 0.0;
		myShaderVals2.delta.m[0][2] = 0.0;
		myShaderVals2.delta.m[1][2] = 0.0;
		myShaderVals2.delta.m[2][2] = 1.0;
		myShaderVals2.delta.m[3][3] = 1.0;

		myShaderVals2.deltaInverse.m[0][0] = 1.0;
		myShaderVals2.deltaInverse.m[1][1] = 1.0;
		myShaderVals2.deltaInverse.m[2][2] = 1.0;
		myShaderVals2.deltaInverse.m[3][3] = 1.0;

		myShaderVals2.eyeOffsetPoseLeft.m[0][0] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[1][1] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[2][2] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[3][3] = 1.0;

		myShaderVals2.eyeOffsetPoseRight.m[0][0] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[1][1] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[2][2] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[3][3] = 1.0;

		myShaderVals2.eyeOffsetPoseLeft.m[0][0] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[1][1] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[2][2] = 1.0;
		myShaderVals2.eyeOffsetPoseLeft.m[3][3] = 1.0;

		myShaderVals2.eyeOffsetPoseRight.m[0][0] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[1][1] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[2][2] = 1.0;
		myShaderVals2.eyeOffsetPoseRight.m[3][3] = 1.0;

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
			dataPtr->delta = myShaderVals2.delta;

			dataPtr->eyeOffsetPoseLeft = myShaderVals2.eyeOffsetPoseLeft;
			dataPtr->eyeOffsetPoseLeftInv = myShaderVals2.eyeOffsetPoseLeft;

			dataPtr->eyeOffsetPoseRight = myShaderVals2.eyeOffsetPoseRight;
			dataPtr->eyeOffsetPoseRightInv = myShaderVals2.eyeOffsetPoseRightInv;

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
	void UseExternalTexture();
	void InitTextureSampler();
	void InitGraphics();// creates the shape to render
	void CleanD3D();
};


