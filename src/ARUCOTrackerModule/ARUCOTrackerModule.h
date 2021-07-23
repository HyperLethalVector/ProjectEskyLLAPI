#include "common_header.h"
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/video/tracking.hpp>
using namespace cv;

typedef void(*FuncPoseReceiveCallback)(int instanceID, float tx, float ty, float tz, float rx, float ry, float rz);
class ARUCOPoseReceiveCallback{
public:
	FuncPoseReceiveCallback callback;
};
class ARUCOTracker {
private:
	bool receivedFirstFrame = false;
public:
	unsigned char* imageName;
	//marker_info
	float marker_length;
	int instanceID;
	int markerID;
	int borderBits;
	cv::Mat markerImg;
	cv::Ptr<cv::aruco::Dictionary> dictionary;
	//
	cv::Mat currentlyProcessedFrame;
	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<cv::Vec3d> rvecs, tvecs;
	cv::Mat camera_matrix;
	cv::Mat dist_coeffs;
	std::vector<ARUCOPoseReceiveCallback*> subscribedPoseReceivers;

	
	//kalman filter
	int nStates = 18;            // the number of states
	int nMeasurements = 6;       // the number of measured states
	int nInputs = 0;             // the number of action control
	double dt = 0.005;           // time between measurements (1/FPS)
	cv::KalmanFilter KF;         // instantiate Kalman Filter
	cv::Mat measurements;
	float* translation = new float[3]{ 0,0,0 };
	float* eulerAngles = new float[3]{ 0,0,0 };
	//poses

	void InitMarker(int dictID, int markerID,float markerWidth) {
		this->markerID = markerID;
		switch (dictID) {
			case 0:
				dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
				break;
			case 1:
				dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
				break;
			case 2:
				dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_50);
				break;
			case 3:
				dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_7X7_50);
				break;
		}
		this->marker_length = markerWidth;

//		initKalmanFilter(KF, nStates, nMeasurements, nInputs, 0.025f);
//		measurements = cv::Mat(nMeasurements, 1, CV_64FC1);
//		measurements.setTo(cv::Scalar(0));
	}
	void subscribeToPoseCallback(FuncPoseReceiveCallback callback) {
		ARUCOPoseReceiveCallback* call = new ARUCOPoseReceiveCallback();
		call->callback = callback;
		subscribedPoseReceivers.push_back(call);
	}
	void InitARUCOTrackerParams(float marker_size, float fx, float fy, float cx, float cy, float d1, float d2, float d3, float d4) {
		marker_length = marker_size;		 
		camera_matrix = (cv::Mat_<double>(3, 3) <<
			fx, 0, cx,
			0, fy, cy, 
			0, 0,  1);
		dist_coeffs = (cv::Mat1d(4, 1) << 
			d1, d2, d3, d4
			);
	}
	void PrintMarker(const char* imageName, int markerID, float markerSize, int borderBits) {
		ostringstream oss;
		try {
			aruco::drawMarker(dictionary, markerID, markerSize, markerImg, borderBits);
			imwrite(imageName, markerImg);
		}
		catch (std::exception& e) {
			Debug::Log(e.what(), Color::Red);
		}

	}
	void fillMeasurements(cv::Mat& measurements,
		float transX, float transY, float transZ, float rotEurX, float rotEurY, float rotEurZ)
	{
		// Convert rotation matrix to euler angles
		// Set measurement to predict
		measurements.at<double>(0) = transX; // x
		measurements.at<double>(1) = transY; // y
		measurements.at<double>(2) = transZ; // z
		measurements.at<double>(3) = rotEurZ;      // roll
		measurements.at<double>(4) = rotEurX;      // pitch
		measurements.at<double>(5) = rotEurY;      // yaw
	}
	bool ProcessImage(unsigned char* imageDataRaw, int totalLength, int imageWidth, int imageHeight, int channels) {

		//Debug::Log("Processing image");
		if (!receivedFirstFrame) {
			//Debug::Log("initializing first frame");
			switch (channels) { 
				case 4:
					currentlyProcessedFrame = cv::Mat(imageHeight, imageWidth, CV_8UC4);
					break;
				case 3:
					currentlyProcessedFrame = cv::Mat(imageHeight, imageWidth, CV_8UC3);
					break;
				case 1:
					currentlyProcessedFrame = cv::Mat(imageHeight, imageWidth, CV_8UC1);
					break;
			}
			receivedFirstFrame = true;
			//Debug::Log("Done Initializing first frame");
		}
		else { 
		///	Debug::Log("Memcpy");
			memcpy(currentlyProcessedFrame.data, imageDataRaw, imageWidth * imageHeight * channels);
		//	Debug::Log("Detecting");
			cv::aruco::detectMarkers(currentlyProcessedFrame, dictionary, corners, ids);
			if (ids.size() > 0) { 
		//		Debug::Log("Processing marker pose");
				//			cv::aruco::drawDetectedMarkers(currentlyProcessedFrame, corners, ids);
				cv::aruco::estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);
				//pose needs a kalman filter, do so for the first detected marker
			//	ostringstream oss;
				// << "Length of vec: " << tvecs.size() << std::endl;
				//Debug::Log(oss.str(), Color::Black);
//				fillMeasurements(measurements, tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2], rvecs[0].val[0], rvecs[0].val[1], rvecs[0].val[2]); //add the measurement to the filter
				for (std::vector<ARUCOPoseReceiveCallback*>::iterator it = subscribedPoseReceivers.begin(); it != subscribedPoseReceivers.end(); ++it) {

					if ((*it) != nullptr) {
						(*it)->callback(instanceID, tvecs[0].val[0], tvecs[0].val[1], tvecs[0].val[2], rvecs[0].val[0], rvecs[0].val[1], rvecs[0].val[2]);
					}
				}
				return true;
			}
			else {
				return false;
			}
		}
	}


};