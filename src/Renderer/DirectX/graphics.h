#pragma once

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
typedef struct ShaderVals {//float is 4 bytes
	DirectX::XMFLOAT4X4 leftUvToRectX;
	DirectX::XMFLOAT4X4 leftUvToRectY;
	DirectX::XMFLOAT4X4 rightUvToRectX;
	DirectX::XMFLOAT4X4 rightUvToRectY;
	DirectX::XMFLOAT4X4 cameraMatrixLeft;
	DirectX::XMFLOAT4X4 cameraMatrixRight;
	DirectX::XMFLOAT4 eyeBordersLeft;
	DirectX::XMFLOAT4 eyeBordersRight;
	DirectX::XMFLOAT4 offsets;
}ShaderVals;
class Graphics {
private:
	IDXGISwapChain *swapchain;
	ID3D11Device *dev;
	ID3D11DeviceContext *devcon; 
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
	int width;
	int height;

	int bufferWidth = 0;
	int bufferHeight = 0;
	bool wmCloseFromUnity = false;
	//std::wstring registeredClassName;
public:

	int unityTextureWidth;
	int unityTextureHeight;
	static DXGI_FORMAT colorFormat;
	static int sampleCount;
	static int descQuality;

	void InitD3D(HWND hWnd);
	void GraphicsRelease();
	void RenderFrame();
	void SetTexturePtrLeft(void* texturePtr);
	void SetTexturePtrRight(void* texturePtr);
	void SetSize(int w, int h) { width = w; height = h; }
	void SetCloseFromUnity(bool closeFromUnity) { wmCloseFromUnity = closeFromUnity;}
	bool GetCloseFromUnity() { return wmCloseFromUnity; }
	void SetInformation(float leftUvToRectX[],// = { 0.0 };
		float leftUvToRectY[],// = { 0.0 };
		float rightUvToRectX[],// = { 0.0 };
		float rightUvToRectY[],// = { 0.0 };
		float CameraMatrixLeft[],// = { 0.0 };
		float CameraMatrixRight[],// = { 0.0 };
		float leftOffset[],// = { 0.0 };
		float rightOffset[],// = { 0.0 };
		float eyeBorders[]) {
		ShaderVals myShaderVals;
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
			}
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
			dataPtr->offsets = myShaderVals.offsets;
			devcon->Unmap(g_pConstantBuffer11, 0);
			devcon->VSSetConstantBuffers(0, 1, &g_pConstantBuffer11);
			devcon->PSSetConstantBuffers(0, 1, &g_pConstantBuffer11);
		}
	}
	//void SetRegistredClass(std::wstring registeredClassName) { this->registeredClassName = registeredClassName; }
	//std::wstring GetRegistredClass() { return registeredClassName; }
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


