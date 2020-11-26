
#pragma warning(push)
#pragma warning(disable : 4005)
#include <stdint.h>
#pragma warning(pop)

#include <iostream>
#include <fstream>
#include <memory>
#include <algorithm>
#include <sstream>
#include "graphics.h"

ID3D11Device *unityDev;
ID3D11DeviceContext *unityDevCon;
static IUnityInterfaces* s_UnityInterfaces = NULL;
static IUnityGraphics* s_Graphics = NULL;
static UnityGfxRenderer s_DeviceType = kUnityGfxRendererNull;  

//for debug messages
typedef void (*FuncPtr) (const char *);
FuncPtr Debug = 0;

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API SetDebugFunction(FuncPtr fp) {
	Debug = fp;
}

void DebugMessage(const char * message) {
	if(Debug){
		Debug(message);
	}
}
///////////////////


extern "C" void	UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginLoad(IUnityInterfaces* unityInterfaces)
{
	s_UnityInterfaces = unityInterfaces;
	s_Graphics = s_UnityInterfaces->Get<IUnityGraphics>();
	s_DeviceType = s_Graphics->GetRenderer();

	if(s_DeviceType == kUnityGfxRendererD3D11){
		IUnityGraphicsD3D11* d3d = s_UnityInterfaces->Get<IUnityGraphicsD3D11>();
		unityDev = d3d->GetDevice();
		unityDev->GetImmediateContext((ID3D11DeviceContext **)&unityDevCon);
	} 
}

extern "C" void UNITY_INTERFACE_EXPORT UNITY_INTERFACE_API UnityPluginUnload()
{
   // s_Graphics->UnregisterDeviceEventCallback(OnGraphicsDeviceEvent);
}

DXGI_FORMAT Graphics::colorFormat = DXGI_FORMAT_R8G8B8A8_UNORM;
int Graphics::sampleCount = 1;
int Graphics::descQuality = 0;

//public methods

void Graphics::InitD3D(HWND hWnd) {
	DXGI_SWAP_CHAIN_DESC scd;

	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));

	scd.BufferCount = 1;
	scd.BufferDesc.Format = colorFormat;
	scd.BufferDesc.Width = width;//perhaps 0 is enough
    scd.BufferDesc.Height = height;//perhaps 0 is enough
	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
	scd.OutputWindow = hWnd;
	scd.SampleDesc.Count = 1;
	scd.SampleDesc.Quality = 0;

	scd.Windowed = TRUE;
	scd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;

//	unityTextureWidth = width;
	//unityTextureHeight = height;

	UINT unityCreationFlags;	
	if(s_DeviceType == kUnityGfxRendererD3D11){
		unityCreationFlags = unityDev->GetCreationFlags();
	} 
		
	D3D11CreateDeviceAndSwapChain(NULL, D3D_DRIVER_TYPE_HARDWARE, NULL, unityCreationFlags, NULL, NULL, D3D11_SDK_VERSION, &scd, &swapchain, &dev, NULL, &devcon);	
	SetBufferRenderTargets();
	SetViewport(width, height);
	
	InitPipeline();
	InitTextureSampler();	
    InitGraphics();
}

void Graphics::RenderFrame() {
	if (pExternalTextureLeft) {
		unityDevCon->CopyResource(pProxyTextureLeft, pExternalTextureLeft);
	}
	if (pExternalTextureRight) {
		unityDevCon->CopyResource(pProxyTextureRight, pExternalTextureRight);
	}

	FLOAT color[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
	devcon->ClearRenderTargetView(backbuffer, color);

	UINT stride = sizeof(VERTEX);
	UINT offset = 0;
	devcon->IASetVertexBuffers(0, 1, &pVBuffer, &stride, &offset);
	devcon->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLESTRIP);
	devcon->Draw(4, 0);
	swapchain->Present(0, 0);
}

void Graphics::GraphicsRelease() {
	CleanD3D();
}

void Graphics::SetTexturePtrLeft(void* texturePtr) {
	pExternalTextureLeft = (ID3D11Texture2D*)texturePtr;
}
void Graphics::SetTexturePtrRight(void* texturePtr) {
	pExternalTextureRight = (ID3D11Texture2D*)texturePtr;
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

	const char* shaders = "\
							Texture2D txDiffuse;\
							SamplerState samLinear;\
							\
							struct VOut\
							{\
								float4 position : SV_POSITION;\
								float2 tex : TEXCOORD;\
							};\
							\
							VOut VShader(float4 position : POSITION, float2 tex : TEXCOORD)\
							{\
								VOut output;\
								\
								output.position = position;\
								output.tex = tex;\
								\
								return output;\
							}\
							\
							\
							float4 PShader(float4 position : SV_POSITION, float2 tex: TEXCOORD) : SV_TARGET\
							{\
								return txDiffuse.Sample( samLinear, tex );\
							}";
	std::ifstream myfile("shaders.shader");
	std::stringstream buffer;
	buffer << myfile.rdbuf();
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
	//D3DCompile(shaders, strlen(shaders), 0, 0, 0, "VShader", "vs_4_0", 0, 0, &VS, 0);
//	D3DCompile(shaders, strlen(shaders), 0, 0, 0, "PShader", "ps_4_0", 0, 0, &PS, 0);


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

	if (FAILED(hr))
	{
	}
	else {
		
	}
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
void Graphics::UseExternalTexture()
{
	if(pExternalTextureLeft){
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
}

void Graphics::InitTextureSampler()
{
	UseExternalTexture();
	
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

	devcon->PSSetShaderResources(0, 1, &pShaderResourceViewLeft);
	devcon->PSSetShaderResources(1, 1, &pShaderResourceViewRight);

//	devcon->PSSetConstantBuffers(0, 1, &myShaderVals);

	devcon->PSSetSamplers(0, 1, &pSamplerState);

	// Set the buffer.

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
	//Release Left resources
	if(pShaderResourceViewLeft){
		pShaderResourceViewLeft->Release();
		pShaderResourceViewLeft = NULL;
	}
	if(pTextureLeft){
		pTextureLeft->Release();
		pTextureLeft = NULL;
	}
	if(pProxyTextureLeft){
		pProxyTextureLeft->Release();
		pProxyTextureLeft = NULL;
	}
	//release right resources
	if (pShaderResourceViewRight) {
		pShaderResourceViewRight->Release();
		pShaderResourceViewRight = NULL;
	}
	if (pTextureRight) {
		pTextureRight->Release();
		pTextureRight = NULL;
	}
	if (pProxyTextureRight) {
		pProxyTextureRight->Release();
		pProxyTextureRight = NULL;
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
	//release the buffer used to upload


	bufferWidth = 0;
	bufferHeight = 0;
}




