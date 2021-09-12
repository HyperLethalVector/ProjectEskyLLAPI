#pragma once 
#include <thread>
#include <d3d11.h>
#include <d3dcompiler.h>
#include <DirectXMath.h>
#include <algorithm>
#include <iterator>
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <opencv2/video/tracking.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h> 
#include <stack>
#include <float.h>
#include <opencv2/core/directx.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>


#pragma comment (lib, "d3d11.lib")
#pragma comment (lib, "d3dcompiler.lib")

#include "IUnityInterface.h"
#include "IUnityGraphics.h"

#include "IUnityGraphicsD3D11.h"
#include "common_header.h" 
#include <string>
#include <DirectXMath.h>
#include <opencv2/video/tracking.hpp>
#include <map>

using namespace cv;
typedef void(*FuncTextureInitializedCallback)(int TextureWidth, int TextureHeight, int textureCount);
typedef void(*FuncReceiveCameraImageCallbackWithID)(int instanceID, unsigned char* info, int lengthofarray, int width, int height, int pixelCount);
class RGBSensorInfoCallback {
public:
	FuncReceiveCameraImageCallbackWithID callbackWithID;
	int instanceID = 0;
};
class RGBSensorModule {
private:
public:
	FuncTextureInitializedCallback textureInitializedCallback = nullptr;
	cv::Mat distCoeffsL;
	cv::Mat intrinsicsL;
	cv::Mat lm1;
	cv::Mat lm2;
	bool hasReceivedCameraStream = false;
	int camID;
	bool stopFlags = false;
	cv::Mat frame;
	cv::Mat RGBAFrame;
	cv::Mat undistorted;
	int textureChannels;
	bool LockImage; 
	std::vector<RGBSensorInfoCallback*> subscribedImageReceivers;
	ID3D11Device* m_Device;
#ifdef __linux__ //OGL

#else
	ID3D11Texture2D* d3dtex;

#endif
	void SetIntrinsicParamters(int cam, float fx,float fy,float cx,float cy, float d1, float d2, float d3, float d4) {
		camID = cam;
		intrinsicsL = (cv::Mat_<double>(3, 3) <<
			fx, 0, cx,
			0, fy, cy, 0, 0, 1);
		distCoeffsL = (cv::Mat1d(4, 1) << d1, d2, d3, d4);
		cv::Mat PP1(3, 4, cv::DataType<float>::type);
		intrinsicsL.copyTo(PP1.rowRange(0, 3).colRange(0, 3));
		cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
		cv::initUndistortRectifyMap(intrinsicsL, distCoeffsL, identity, PP1, cv::Size(1280, 720), CV_32FC1, lm1, lm2);
		stopFlags = false;
	}

	void SetTexturePointer(void* textureHandle) {
#ifdef __linux__ //OGL

#else //DX
		d3dtex = (ID3D11Texture2D*)textureHandle;
#endif
	}
	void UpdatecameraTextureGPU() {
#ifdef __linux__ //OGL

#else
		if (m_Device != nullptr) {
			if (hasReceivedCameraStream) {
				//Debug::Log("Pumping camera to GPU");
				ID3D11DeviceContext* ctx = NULL;
				m_Device->GetImmediateContext(&ctx);
				LockImage = true;
				if (d3dtex != nullptr) {
					if (ctx != nullptr) { 
						try {
							ctx->UpdateSubresource(d3dtex, 0, 0, RGBAFrame.data, (UINT)RGBAFrame.cols * RGBAFrame.channels(), (UINT)RGBAFrame.total());
						}
						catch (exception& e) {
							Debug::Log(e.what());
						}
					}
				}
				LockImage = false;
				ctx->Release();
			}
		}
		else {
		}
#endif
	}
	void SubscribeReceiver(int callbackID, FuncReceiveCameraImageCallbackWithID callback) {
		RGBSensorInfoCallback* c = new RGBSensorInfoCallback();
		c->instanceID = callbackID;
		c->callbackWithID = callback;
		subscribedImageReceivers.push_back(c);
	}
	void OpenCameraThread() {

		VideoCapture cap(camID); // open the default camera
		cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
		cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

		if (!cap.isOpened()) { 
			Debug::Log("Couldn't Open Sensor");
			return;
		}// check if we succeeded

		Debug::Log("Opened Sensor");
		while (!stopFlags)
		{
			Debug::Log("Captured Sensor Frame");
			cap >> frame;
			 // get a new frame from cameras

			if (!hasReceivedCameraStream) {
				undistorted = frame.clone();
				RGBAFrame = cv::Mat(frame.size(), CV_8UC4);
			}
			cv::Exception* ee;

			try {
				cv::remap(frame, undistorted, lm1, lm2, cv::INTER_LINEAR);
				cv::cvtColor(undistorted, RGBAFrame, cv::COLOR_BGR2BGRA, 4);
			}
			catch (cv::Exception& e) {
				ee = &e;
				Debug::Log(e.what(), Color::Red);
				stopFlags = true;
				break;
			}
			if (!hasReceivedCameraStream) {
				textureChannels = 4;
				if (textureInitializedCallback != nullptr) {
					textureInitializedCallback(RGBAFrame.cols, RGBAFrame.rows, 4);
				}
				hasReceivedCameraStream = true;
			}
			try {
				for (std::vector<RGBSensorInfoCallback*>::iterator it = subscribedImageReceivers.begin(); it != subscribedImageReceivers.end(); ++it) {
					if ((*it) != nullptr) {
						if ((*it)->callbackWithID != nullptr)
							(*it)->callbackWithID((*it)->instanceID, undistorted.data, undistorted.rows * undistorted.cols, undistorted.cols, undistorted.rows, undistorted.channels());
					}
				}
			}
			catch (Exception& e) {
				Debug::Log("Blegh", Color::Red);
			}
		}
		Debug::Log("Closing Sensor");
		cap.release();
		Debug::Log("Closed Sensor");
	}
};