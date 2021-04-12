#include <glm/common.hpp>

#include <cmath>
#include <glm/mat4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#pragma warning(push)
#pragma warning(disable : 4005)
#include <stdint.h>
#pragma warning(pop)
#include <stdio.h> 
#include <direct.h>
#include <iostream>
#include <fstream>
#include <memory>
#include <algorithm>
#include <sstream>
#include "graphics.h"

#ifdef __linux__ //OGL

#else
#include <Windows.h>
#endif
ID3D11Device* unityDev;
ID3D11DeviceContext* unityDevCon;
static IUnityInterfaces* s_UnityInterfaces = NULL;
static IUnityGraphics* s_Graphics = NULL;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;
struct FTextureShareReceiverData
{
	ID3D11Texture2D* Texture = nullptr;
	ID3D11ShaderResourceView* TextureSRV = nullptr;
};
FTextureShareReceiverData ShareData1[2] = { FTextureShareReceiverData(), FTextureShareReceiverData() };

std::wstring ShareName1 = L"EskyTextures";
std::wstring ReceiveTextureNames[2] = { L"LeftEye" , L"RightEye" };
static FTextureShareD3D11Client* TextureShareClient = nullptr;
 
enum ESharedTextureName
{
	// Read textures
	LeftEye = 0,
	RightEye,
	COUNT
};

float* DeltaPoseIdentity = new float[] {1, 0, 0, 0,
0, 1, 0, 0,
0, 0, 1, 0,
0, 0, 0, 1};

typedef void (*FuncPtr) (const wchar_t*);
FuncPtr Debug = 0;
//for debug messages
void DebugMessage(const wchar_t* message){
	if (Debug) {
		Debug(message);
	}
}

extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
	/*s_UnityInterfaces = unityInterfaces;
	s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
	s_DeviceType = s_Graphics->GetRenderer();

	if(s_DeviceType == kUnityGfxRendererD3D11){
		IUnityGraphicsD3D11* d3d = s_UnityInterfaces->Get<IUnityGraphicsD3D11>();
		unityDev = d3d->GetDevice();
		unityDev->GetImmediateContext((ID3D11DeviceContext **)&unityDevCon);
	} */
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetDebugFunction(int WindowID, FuncPtr fp) {
	Debug = fp;
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API FreeDebugFunction(int WindowID) {
	Debug = NULL;
}
extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
}
bool Graphics::UpdateTextureShareClient()
{
	bool bResult = false;
	if (TextureShareClient && TextureShareClient->BeginFrame_RenderThread(ShareName1))
	{
		// Receive all textures
		for (int i = 0; i < ESharedTextureName::COUNT; i++)
		{
			if (TextureShareClient->ReadTextureFrame_RenderThread(ShareName1, ReceiveTextureNames[i], &ShareData1[i].Texture, &ShareData1[i].TextureSRV))
			{
				bResult = true;
			}
		}
		// Get frame data
		FTextureShareSDKAdditionalData FrameData;
		TextureShareClient->ReadAdditionalData(ShareName1, &FrameData);
		TextureShareClient->EndFrame_RenderThread(ShareName1);
	}
	return bResult;
}
DXGI_FORMAT Graphics::colorFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
int Graphics::sampleCount = 1;
int Graphics::descQuality = 0;
void Graphics::InitD3D(HWND hWnd) {
	DXGI_SWAP_CHAIN_DESC scd;
	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));
	scd.BufferCount = 1;
	scd.BufferDesc.Format = colorFormat;
	scd.BufferDesc.Width = width;
    scd.BufferDesc.Height = height;
	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	scd.OutputWindow = hWnd; 
	scd.SampleDesc.Count = 1;
	scd.SampleDesc.Quality = 0;

	scd.Windowed = TRUE;
	scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
	std::wstring s = L"Before using the unity flags";
	
	
	DebugMessage(s.c_str());
	UINT unityCreationFlags;	
	if(s_DeviceType == kUnityGfxRendererD3D11){
		unityCreationFlags = unityDev->GetCreationFlags();
	}
	else {
		unityCreationFlags = 0;
	}
	DebugMessage(L"Creating the swap chain");
	D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, unityCreationFlags, NULL, NULL, D3D11_SDK_VERSION, &scd, &swapchain, &dev, NULL, &devcon);	
	SetBufferRenderTargets(); 
	
	doesExit = false;
	graphicsRender = true;
	wmCloseFromUnity = false;
	hasExitedGraphicsThread = false;
	hasClosedExternalWindow = false;
	hasClosedWindow = false;

	DebugMessage(L"Setting the viewport");
	SetViewport(width, height);	
	DebugMessage(L"Init Pipeline");
	InitPipeline();
	DebugMessage(L"Initialize texture sampler");
	InitTextureSampler();	
	DebugMessage(L"Init Graphics");

	InitGraphics();

}

double last_ms_graphics = 0;
void Graphics::GraphicsBackgroundThreadRenderFrame() {
	graphicsRender = true;
	wmCloseFromUnity = false;
	hasExitedGraphicsThread = false;
	hasClosedExternalWindow = false;
	hasClosedWindow = false;
	lockRenderingFrame = false;
	RenderLock = false;
	DebugMessage(L"Creating texture share client");
	if (!TextureShareClient) {
		TextureShareClient = new FTextureShareD3D11Client(dev);
	}
	// Create & initialize share1
	DebugMessage(L"Creating texture ");
	TextureShareClient->CreateShare(ShareName1);
	// Register receive textures:
	TextureShareClient->RegisterTexture(ShareName1, ReceiveTextureNames[ESharedTextureName::LeftEye], ETextureShareSurfaceOp::Read);
	TextureShareClient->RegisterTexture(ShareName1, ReceiveTextureNames[ESharedTextureName::RightEye], ETextureShareSurfaceOp::Read);
	// Begin share session
	TextureShareClient->BeginSession(ShareName1);
	devcon->PSSetShaderResources(0, 1, &ShareData1[0].TextureSRV);
	devcon->PSSetShaderResources(1, 1, &ShareData1[1].TextureSRV);
	
	while (graphicsRender) {
		if (!lockRenderingFrame && !RenderLock) {
			auto now = std::chrono::system_clock::now().time_since_epoch();
			double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
			if (updateDeltaPoseOnGraphicsThread) {
				updateDeltaPoseOnGraphicsThread = false;
				if (g_pConstantBuffer11_2) {
					D3D11_MAPPED_SUBRESOURCE mappedResource;
					devcon->Map(g_pConstantBuffer11_2, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
					ShaderVals2* dataPtr = (ShaderVals2*)mappedResource.pData;
					dataPtr->deltaPoseLeft = myShaderVals2.deltaPoseLeft;
					dataPtr->deltaPoseRight = myShaderVals2.deltaPoseRight;
					dataPtr->toggleConfigs = myShaderVals2.toggleConfigs;
					devcon->Unmap(g_pConstantBuffer11_2, 0);
					devcon->VSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2);
					devcon->PSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2);
				} 
			}
			if (UpdateTextureShareClient()) {//we received a new frame, set the shader resources
				if (!receivedTextureShareResource) {
					receivedTextureShareResource = true;
					devcon->PSSetShaderResources(0, 1, &ShareData1[0].TextureSRV);
					devcon->PSSetShaderResources(1, 1, &ShareData1[1].TextureSRV);
				}
				else {
					if (renderedFrameCallback) {
						renderedFrameCallback();
						if (g_pConstantBuffer11_2) {
							for (int x = 0; x < 4; x++) {
								for (int y = 0; y < 4; y++) {
									myShaderVals2.deltaPoseLeft.m[x][y] = DeltaPoseIdentity[y * 4 + x];
									myShaderVals2.deltaPoseRight.m[x][y] = DeltaPoseIdentity[y * 4 + x];
								}
							}
							D3D11_MAPPED_SUBRESOURCE mappedResource;
							devcon->Map(g_pConstantBuffer11_2, 0, D3D11_MAP_WRITE_DISCARD, 0, &mappedResource);
							ShaderVals2* dataPtr = (ShaderVals2*)mappedResource.pData;
							dataPtr->deltaPoseLeft = myShaderVals2.deltaPoseLeft;
							dataPtr->deltaPoseRight = myShaderVals2.deltaPoseRight;
							dataPtr->toggleConfigs = myShaderVals2.toggleConfigs;
							devcon->Unmap(g_pConstantBuffer11_2, 0);
							devcon->VSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2);
							devcon->PSSetConstantBuffers(1, 1, &g_pConstantBuffer11_2);
						}
					}
				}
			}
			FLOAT color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
			devcon->ClearRenderTargetView(backbuffer, color);
			UINT stride = sizeof(VERTEX);
			UINT offset = 0;
			devcon->IASetVertexBuffers(0, 1, &pVBuffer, &stride, &offset);
			devcon->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
			devcon->Draw(4, 0);
			swapchain->Present(0, 0);
			if (wmCloseFromUnity) {
				DebugMessage(L"Something closed me from above");
				graphicsRender = false;
				break;
			}
			else {
				now = std::chrono::system_clock::now().time_since_epoch();
				double timeAtEnd = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
				float timeDeltaToProcessPose = static_cast<float>(max(0.0, ((now_ms - last_ms_graphics) / 1000.0)));
				if (timeDeltaToProcessPose < 0.008f) {//we want to ensure our process time is a minimum of 120Hz
					float timeToWait = 0.008f - timeDeltaToProcessPose;
					Sleep(timeToWait * 1000);
				}
				else {
				}
				last_ms_graphics = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
			}
		}

	}
	if (TextureShareClient)
	{
		DebugMessage(L"Closing the texture share client");
		TextureShareClient->EndSession(ShareName1);
		DebugMessage(L"Ended Session");
		TextureShareClient->DeleteShare(ShareName1);
		DebugMessage(L"Deleted Share");
		delete TextureShareClient;
		TextureShareClient = nullptr;
		DebugMessage(L"Deleted Texture Share");
	}
} 
void Graphics::GraphicsRelease() {
	CleanD3D();
	hasClosedExternalWindow = true; 
}

void Graphics::SetTexturePtrLeft(void* texturePtr) {
	initializedWithTextures = true;
	pExternalTextureLeft = (ID3D11Texture2D*)texturePtr;
}
void Graphics::SetTexturePtrRight(void* texturePtr) {
	initializedWithTextures = true;
	pExternalTextureRight = (ID3D11Texture2D*)texturePtr;
}
void Graphics::SetTexturePtrLuTs(void* texturePtrLeft, void* texturePtrRight) {
	pExternalTextureLeftLuT = (ID3D11Texture2D*)texturePtrLeft;
	pExternalTextureRightLuT = (ID3D11Texture2D*)texturePtrRight;
	updateLuT = true;
}
//private methods

void Graphics::SetViewport(int width, int height) {
	D3D11_VIEWPORT viewport;
	ZeroMemory(&viewport, sizeof(D3D11_VIEWPORT));

	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.Width = width;
	viewport.Height = height;

	devcon->RSSetViewports(1, &viewport);
}

void Graphics::SetBufferRenderTargets() {

	ID3D11Texture2D *pBackBuffer;
	swapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);
	dev->CreateRenderTargetView(pBackBuffer, NULL, &backbuffer);
	pBackBuffer->Release();
	devcon->OMSetRenderTargets(1, &backbuffer, NULL);
}

void Graphics::InitPipeline()
{
	ID3D10Blob* VS, * PS;
	DebugMessage(L"Finding Shader");
	char buff[FILENAME_MAX];
	const char *s = _getcwd(buff, FILENAME_MAX);
	// required size
	int nChars = MultiByteToWideChar(CP_ACP, 0, s, -1, NULL, 0); 
	// allocate it
	WCHAR* pwcsName = new WCHAR[nChars];
	MultiByteToWideChar(CP_ACP, 0, s, -1, (LPWSTR)pwcsName, nChars);
	DebugMessage(pwcsName);
	std::ifstream myfile("shaders.shader");
	DebugMessage(L"Found Shader!");
	std::stringstream buffer;

	buffer << myfile.rdbuf();
	DebugMessage(L"Read Shader!");
	ID3DBlob** errorBlob = nullptr;
	HRESULT hrVS = D3DCompile(buffer.str().c_str(), strlen(buffer.str().c_str()), 0, 0, 0, "VShader", "vs_4_0", 0, 0, &VS, errorBlob);
	if (FAILED(hrVS)) {
		if (errorBlob) {
		}
	}
	HRESULT hrPS = D3DCompile(buffer.str().c_str(), strlen(buffer.str().c_str()), 0, 0, 0, "PShader", "ps_4_0", 0, 0, &PS, errorBlob);
	if (FAILED(hrVS)) {
		if (errorBlob) {
		}
	}
	dev->CreateVertexShader(VS->GetBufferPointer(), VS->GetBufferSize(), NULL, &pVS);
    dev->CreatePixelShader(PS->GetBufferPointer(), PS->GetBufferSize(), NULL, &pPS);

	devcon->VSSetShader(pVS, 0, 0);
    devcon->PSSetShader(pPS, 0, 0);

	D3D11_INPUT_ELEMENT_DESC ied[] = {
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 }		
	};
	dev->CreateInputLayout(ied, 2, VS->GetBufferPointer(), VS->GetBufferSize(), &pLayout);
	devcon->IASetInputLayout(pLayout);
	D3D11_BUFFER_DESC cbDesc;
	cbDesc.Usage = D3D11_USAGE_DYNAMIC;
	cbDesc.ByteWidth = sizeof(ShaderVals);
	cbDesc.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cbDesc.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cbDesc.MiscFlags = 0;
	cbDesc.StructureByteStride = 0;
	// Create the buffer.
	HRESULT hr = dev->CreateBuffer(&cbDesc, NULL, &g_pConstantBuffer11); 
	D3D11_BUFFER_DESC cbDesc2;
	cbDesc2.Usage = D3D11_USAGE_DYNAMIC;
	cbDesc2.ByteWidth = sizeof(ShaderVals2);
	cbDesc2.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cbDesc2.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cbDesc2.MiscFlags = 0;
	cbDesc2.StructureByteStride = 0;
	// Create the buffer.
	HRESULT hr2 = dev->CreateBuffer(&cbDesc2, NULL, &g_pConstantBuffer11_2);
}

void Graphics::CreateProxyTextureLeft(){
	D3D11_TEXTURE2D_DESC desc;
	ZeroMemory(&desc, sizeof(D3D11_TEXTURE2D_DESC));
	desc.Width = unityTextureWidth;
	desc.Height = unityTextureHeight;
	desc.MipLevels = 1;
	desc.ArraySize = 1;
	desc.Format = colorFormat;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Usage = D3D11_USAGE_DEFAULT;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	desc.CPUAccessFlags = 0;
	desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED;
	unityDev->CreateTexture2D(&desc, NULL, &pProxyTextureLeft);
}
void Graphics::CreateProxyTextureRight() {
	D3D11_TEXTURE2D_DESC desc;
	ZeroMemory(&desc, sizeof(D3D11_TEXTURE2D_DESC));
	desc.Width = unityTextureWidth;
	desc.Height = unityTextureHeight;
	desc.MipLevels = 1;
	desc.ArraySize = 1;
	desc.Format = colorFormat;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Usage = D3D11_USAGE_DEFAULT;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	desc.CPUAccessFlags = 0;
	desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED;
	unityDev->CreateTexture2D(&desc, NULL, &pProxyTextureRight);
}
void Graphics::CreateProxyLuT() {
	D3D11_TEXTURE2D_DESC desc;
	ZeroMemory(&desc, sizeof(D3D11_TEXTURE2D_DESC));
	desc.Width = unityTextureWidth;
	desc.Height = unityTextureHeight;
	desc.MipLevels = 1;
	desc.ArraySize = 1;
	desc.Format = colorFormat;
	desc.SampleDesc.Count = 1;
	desc.SampleDesc.Quality = 0;
	desc.Usage = D3D11_USAGE_DEFAULT;
	desc.BindFlags = D3D11_BIND_SHADER_RESOURCE;
	desc.CPUAccessFlags = 0;
	desc.MiscFlags = D3D11_RESOURCE_MISC_SHARED;
	unityDev->CreateTexture2D(&desc, NULL, &pProxyTextureLeftLuT);
	unityDev->CreateTexture2D(&desc, NULL, &pProxyTextureRightLuT);
}
void Graphics::UseExternalTexture()
{
	if(pExternalTextureLeft){
		DebugMessage(L"Using External Texture Left");
		if(s_DeviceType == kUnityGfxRendererD3D11){
			CreateProxyTextureLeft();
			unityDevCon->CopyResource(pProxyTextureLeft, pExternalTextureLeft);
			IDXGIResource* pOtherResource(NULL);
			HRESULT hr = pProxyTextureLeft->QueryInterface( __uuidof(IDXGIResource), (void**)&pOtherResource);
			HANDLE sharedHandle;
			pOtherResource->GetSharedHandle(&sharedHandle);
			dev->OpenSharedResource(sharedHandle, __uuidof(ID3D11Texture2D), (LPVOID*)&pTextureLeft);
		} 
		dev->CreateShaderResourceView(pTextureLeft, NULL, &pShaderResourceViewLeft);
	}
	if (pExternalTextureRight) {
		DebugMessage(L"Using External Texture Right");
		if (s_DeviceType == kUnityGfxRendererD3D11) {
			CreateProxyTextureRight();

			unityDevCon->CopyResource(pProxyTextureRight, pExternalTextureRight);

			IDXGIResource* pOtherResource(NULL);
			HRESULT hr = pProxyTextureRight->QueryInterface(__uuidof(IDXGIResource), (void**)&pOtherResource);
			HANDLE sharedHandle;
			pOtherResource->GetSharedHandle(&sharedHandle);
			dev->OpenSharedResource(sharedHandle, __uuidof(ID3D11Texture2D), (LPVOID*)&pTextureRight);
		}
		dev->CreateShaderResourceView(pTextureRight, NULL, &pShaderResourceViewRight);
	}
	if (pExternalTextureLeftLuT && pExternalTextureRightLuT) {
		DebugMessage(L"Using External LuT");
		if (s_DeviceType == kUnityGfxRendererD3D11) {
			CreateProxyLuT();

			unityDevCon->CopyResource(pProxyTextureRightLuT, pExternalTextureRightLuT);
			unityDevCon->CopyResource(pProxyTextureLeftLuT, pExternalTextureLeftLuT);

			IDXGIResource* pOtherResourceR(NULL);
			IDXGIResource* pOtherResourceL(NULL);

			HRESULT hrR = pProxyTextureRightLuT->QueryInterface(__uuidof(IDXGIResource), (void**)&pOtherResourceR);
			HRESULT hrL = pProxyTextureLeftLuT->QueryInterface(__uuidof(IDXGIResource), (void**)&pOtherResourceL);
			HANDLE sharedHandleR;
			HANDLE sharedHandleL;
			pOtherResourceR->GetSharedHandle(&sharedHandleR);
			pOtherResourceL->GetSharedHandle(&sharedHandleL);
			dev->OpenSharedResource(sharedHandleR, __uuidof(ID3D11Texture2D), (LPVOID*)&pTextureRightLuT);
			dev->OpenSharedResource(sharedHandleL, __uuidof(ID3D11Texture2D), (LPVOID*)&pTextureLeftLuT);
		}
		dev->CreateShaderResourceView(pTextureRightLuT, NULL, &pShaderResourceViewRightLuT);
		dev->CreateShaderResourceView(pTextureLeftLuT, NULL, &pShaderResourceViewLeftLuT);
	}
}

void Graphics::InitTextureSampler()
{
	DebugMessage(L"Trying (but should fail) to use external texture");
	if (initializedWithTextures) {
		UseExternalTexture();
	}
	DebugMessage(L"Creating Sampler");
	D3D11_SAMPLER_DESC sampDesc;
	ZeroMemory( &sampDesc, sizeof(sampDesc) );

	sampDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
    sampDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
    sampDesc.ComparisonFunc = D3D11_COMPARISON_NEVER;
    sampDesc.MinLOD = 0;
    sampDesc.MaxLOD = D3D11_FLOAT32_MAX;

	dev->CreateSamplerState(&sampDesc, &pSamplerState);
	DebugMessage(L"Shouldn't be trying to set the resource views");
	if (initializedWithTextures) {
		devcon->PSSetShaderResources(0, 1, &pShaderResourceViewLeft);
		devcon->PSSetShaderResources(1, 1, &pShaderResourceViewRight);
		devcon->PSSetShaderResources(2, 1, &pShaderResourceViewLeftLuT);
		devcon->PSSetShaderResources(3, 1, &pShaderResourceViewRightLuT);
	}
	DebugMessage(L"Setting the samplers");
	devcon->PSSetSamplers(0, 1, &pSamplerState);
}

void Graphics::InitGraphics()
{	 
	VERTEX OurVertices[] =
	{
		{-1, 1, 0, 0, 1},
		{ 1, 1, 0, 1, 1},
		{-1,-1, 0, 0, 0},
		{ 1,-1, 0, 1, 0},
	};

	D3D11_BUFFER_DESC bd;
	ZeroMemory(&bd, sizeof(bd));

	bd.Usage = D3D11_USAGE_DYNAMIC; 
	bd.ByteWidth = sizeof(VERTEX) * 4;
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	dev->CreateBuffer(&bd, NULL, &pVBuffer);
	D3D11_MAPPED_SUBRESOURCE ms;
	devcon->Map(pVBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);
	memcpy(ms.pData, OurVertices, sizeof(OurVertices)); 
	devcon->Unmap(pVBuffer, NULL); 
}

void Graphics::CleanD3D()
{

	if(swapchain){
		swapchain->SetFullscreenState(FALSE, NULL);
	}
	if(pLayout){
		pLayout->Release();
		pVS->Release();
		pPS->Release();
		pLayout = NULL;
		pVS = NULL;
		pPS = NULL;
	}
	if(pVBuffer){
		pVBuffer->Release();
		pVBuffer = NULL;
	}
	if (g_pConstantBuffer11) {
		g_pConstantBuffer11->Release();
		g_pConstantBuffer11 = NULL;
	}
	if (g_pConstantBuffer11_2) {
		g_pConstantBuffer11_2->Release();
		g_pConstantBuffer11_2 = NULL;
	}
	
	if (initializedWithTextures) {
		if (pTextureLeft) {
			pTextureLeft->Release();
			pTextureLeft = NULL;
		}
		if (pTextureLeftLuT) {
			pTextureLeftLuT->Release();
			pTextureLeftLuT = NULL;
		}
		if (pProxyTextureLeft) {
			pProxyTextureLeft->Release();
			pProxyTextureLeft = NULL;
		}
		if (pProxyTextureLeftLuT) {
			pProxyTextureLeftLuT->Release();
			pProxyTextureLeftLuT = NULL;
		}
		if (pTextureRight) {
			pTextureRight->Release();
			pTextureRight = NULL;
		}
		if (pTextureRightLuT) {
			pTextureRightLuT->Release();
			pTextureRightLuT = NULL;
		}
		if (pProxyTextureRight) {
			pProxyTextureRight->Release();
			pProxyTextureRight = NULL;
		}
		if (pProxyTextureRightLuT) {
			pProxyTextureRightLuT->Release();
			pProxyTextureRightLuT = NULL;
		}
		if (pShaderResourceViewLeft) {
			pShaderResourceViewLeft->Release();
			pShaderResourceViewLeft = NULL;
		}
		if (pShaderResourceViewLeftLuT) {
			pShaderResourceViewLeftLuT->Release();
			pShaderResourceViewLeftLuT = NULL;
		}
		if (pShaderResourceViewRight) {
			pShaderResourceViewRight->Release();
			pShaderResourceViewRight = NULL;
		}
		if (pShaderResourceViewRightLuT) {
			pShaderResourceViewRightLuT->Release();
			pShaderResourceViewRightLuT = NULL;
		}
	}
	
	if(pSamplerState){
		pSamplerState->Release();
		pSamplerState = NULL;
	}
	if(backbuffer){
		backbuffer->Release();
		backbuffer = NULL;
	}

	if(swapchain){
		swapchain->Release();
		dev->Release();
		devcon->Release();
		swapchain = NULL;
		dev = NULL;
		devcon = NULL;
	}
	bufferWidth = 0;
	bufferHeight = 0;
}




