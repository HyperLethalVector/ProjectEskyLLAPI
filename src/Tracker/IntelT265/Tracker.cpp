#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>
#include <stack>
#include <float.h>
#include <opencv2/core/directx.hpp>
#include <d3d11.h>
#include "common_header.h"
#include "IUnityGraphicsD3D11.h"
#include "IUnityInterface.h"
#include "IUnityGraphics.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "Serial.h"
class TrackerObject {
public:
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    typedef void(*FuncCallBack4)(string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
    typedef void(*FuncTextureInitializedCallback)(int TextureWidth, int TextureHeight, int textureCount, float fx, float fy, float cx, float cy, float fovx, float fovy, float focalLength);
    rs2_intrinsics intrinsics;

    FuncTextureInitializedCallback textureInitializedCallback = nullptr;
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;
    FuncCallBack4 callbackObjectPoseReceived = nullptr;
    ID3D11Device* m_Device;
    ID3D11Texture2D* d3dtex;
    bool LockImage = false;
    float* pose = new float[] {0, 0, 0, 0, 0, 0, 0};
    float* poseFromWorldToMap = new float[] {0, 0, 0, 0, 0, 0, 0};
    //get intrinsics as float array
    // the fisheye mat
    cv::Mat fisheye_mat;
    cv::Mat fisheye_mat_color;
    cv::Mat fisheye_mat_color_undistort;
    cv::Mat lm1, lm2;
    rs2_pose object_in_world_pose_frame;
    bool hasLocalized = false;
    unsigned char* fileLocation;
    int textureWidth = 0;
    int textureHeight = 0;
    int textureChannels = 0;
    bool usesIntegrator = false;
    map<string, rs2_pose> posesToUpdate;
    cv::Mat distCoeffsL;
    cv::Mat intrinsicsL;
    bool hasReceivedCameraStream = false;
    char* com_port = "\\\\.\\COM5";
    DWORD COM_BAUD_RATE = CBR_9600;
    SimpleSerial* Serial;
    inline rs2_quaternion quaternion_exp(rs2_vector v)
    {
        float x = v.x / 2, y = v.y / 2, z = v.z / 2, th2, th = sqrtf(th2 = x * x + y * y + z * z);
        float c = cosf(th), s = th2 < sqrtf(120 * FLT_EPSILON) ? 1 - th2 / 6 : sinf(th) / th;
        rs2_quaternion Q = { s * x, s * y, s * z, c };
        return Q;
    }

    std::vector<uint8_t> bytes_from_raw_file(const std::string& filename)
    {
        std::ifstream file(filename.c_str(), std::ios::binary);
        if (!file.good())
            throw std::runtime_error("Invalid binary file specified. Verify the source path and location permissions");

        // Determine the file length
        file.seekg(0, std::ios_base::end);
        std::size_t size = file.tellg();
        if (!size)
            throw std::runtime_error("Invalid binary file -zero-size");
        file.seekg(0, std::ios_base::beg);

        // Create a vector to store the data
        std::vector<uint8_t> v(size);

        // Load the data
        file.read((char*)&v[0], size);

        return v;
    }

    inline rs2_quaternion quaternion_multiply(rs2_quaternion a, rs2_quaternion b)
    {
        rs2_quaternion Q = {
            a.x * b.w + a.w * b.x - a.z * b.y + a.y * b.z,
            a.y * b.w + a.z * b.x + a.w * b.y - a.x * b.z,
            a.z * b.w - a.y * b.x + a.x * b.y + a.w * b.z,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
        return Q;
    }

    rs2_pose predict_pose(rs2_pose& pose, float dt_s)
    {
        rs2_pose P = pose;
        P.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
        P.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
        P.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
        rs2_vector W = {
                dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
                dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
                dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
        };
        P.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
        return P;
    }
    bool DoExit3 = false;
    void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
    {
        std::ofstream file(filename, std::ios::binary | std::ios::trunc);
        if (!file.good()) {
            Debug::Log("Invalid binary file specified. Verify the target path and location permissions");
        }
        file.write((char*)bytes.data(), bytes.size());
    }
    void UpdatecameraTextureGPU() {

        if (m_Device != nullptr) {
            if (hasReceivedCameraStream) {
                ID3D11DeviceContext* ctx = NULL;
                m_Device->GetImmediateContext(&ctx);
                LockImage = true;
                if (d3dtex != nullptr) {
                    ctx->UpdateSubresource(d3dtex, 0, 0, fisheye_mat_color_undistort.data, fisheye_mat_color_undistort.step[0], (UINT)fisheye_mat_color_undistort.total());
                }
                LockImage = false;
                ctx->Release();
            }
        }
    }
    // Create a configuration for configuring the pipeline with a non default profile

    std::vector<unsigned char> myMapData;
    bool mapDataLoaded = false;
    void StopTracking() {
        DoExit3 = true;
    }
    bool doSetOriginMapInfo = false;
    void SetOrigin() {
        doSetOriginMapInfo = true;
    }
    bool grabOriginPose = false;
    void GrabObjectPose(string objectID) {
        if (hasLocalized) {
            Debug::Log("Was able to call back, but needs to fire events locally");
            grabOriginPose = true;
        }
        else {
            Debug::Log("ERROR: The map hasn't localized yet!", Color::Red);
        }
    }
    void ClearObjectPoses() {
        posesToUpdate.clear();
    }
    void SetObjectPose(const char* objReference, float tx, float ty, float tz, float qx, float qy, float qz, float qw) {
        Debug::Log("Attempting to set rs2pose");
        Debug::Log(objReference);

        poseRefObjct.translation.x = tx;
        poseRefObjct.translation.y = ty;
        poseRefObjct.translation.z = tz;

        poseRefObjct.rotation.x = qx;
        poseRefObjct.rotation.y = qy;
        poseRefObjct.rotation.z = qz;
        poseRefObjct.rotation.w = qw;
        Debug::Log("Should try to save the info now");
        posesToUpdate.insert(std::pair<string, rs2_pose>(objReference, poseRefObjct));

    }
    void ProcessPoseCache() {

    }
    void SetTexturePointer(void* textureHandle) {
        d3dtex = (ID3D11Texture2D*)textureHandle;
    }
    void GrabMap() {
        if (callbackBinaryMap != nullptr) {
            Debug::Log("Attempting to obtain localization map", Color::Red);
            shouldGrabMap = true;
        }
        else {
            Debug::Log("ERROR: map callback doesn't exist? How the F*@(#& did this happen when it's supposed to on awake? Zzzz", Color::Red);
        }
    }
    bool isFirst = true;
    rs2_pose poseRefObjct;
    bool shouldRestart = false;
    std::vector<uint8_t> mapDataToLoad;
    bool shouldLoadMap = false;
    bool shouldGrabMap = false;
    void ImportMap(uint8_t* mapData, int length) {
            Debug::Log("Loading map!", Color::Green);
            shouldLoadMap = true;
    }
    void SetComPortString(int port) {
        switch (port) {
        case 0:
            com_port = "\\\\.\\COM0";
            break;
        case 1:
            com_port = "\\\\.\\COM1";
            break;
        case 2:
            com_port = "\\\\.\\COM2";
            break;
        case 3:
            com_port = "\\\\.\\COM3";
            break;
        case 4:
            com_port = "\\\\.\\COM4";
            break;
        case 5:
            com_port = "\\\\.\\COM5";
            break;
        case 6:
            com_port = "\\\\.\\COM6";
            break;
        case 7:
            com_port = "\\\\.\\COM7";
            break;
        case 8:
            com_port = "\\\\.\\COM8";
            break;
        }

    }
    bool shouldUploadData = false;
    void DoFunctionTracking() {
        if (usesIntegrator) {
            Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
            if (Serial->WriteSerialPort("r\r\n")) {
                Debug::Log("Restarted the t265/1",Color::Green);  
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        }
        while (!DoExit3) {
            try {
                hasReceivedCameraStream = false;
                rs2::pipeline pipe;
                rs2::config cfg;
                rs2::pipeline_profile myProf;
                cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
                cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE,1);
                cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE,2);
                rs2::pose_sensor tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
                
                tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
                    if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                        hasLocalized = true;
                        if (callbackLocalization != nullptr) {
                            callbackLocalization(1);
                        }
                        else {
                            Debug::Log("The callback doesn't exist, wtf?",Color::Red);
                        }
                    }
                    });
                //First, should we upload the map?
                if (shouldUploadData) {
                    shouldUploadData = false;
                    try {
                        Debug::Log("Loading Map");

                        if (tm_sensor.import_localization_map(bytes_from_raw_file("temp.raw"))) {
                            ostringstream oss;
                            oss << "Map loaded from input data succesfully!" << std::endl;
                            Debug::Log(oss.str(), Color::Green);
                        }  
                        else {
                            ostringstream oss;
                            oss << "Issue loading the map??" << std::endl;
                            Debug::Log(oss.str(), Color::Green);
                        }
                    }
                    catch (std::exception e) {
                        ostringstream oss;
                        oss << "Couldn't Load Map: " << e.what() << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                    }
                } 
                //Then let's initialize the sensor and begin tracking
               

                myProf = pipe.start(cfg, [&](rs2::frame frame) {
                    if (auto fs = frame.as<rs2::pose_frame>()) {
                        rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>();
                        rs2_pose pose_data = pose_frame.get_pose_data();
                        auto now = std::chrono::system_clock::now().time_since_epoch();
                        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                        double pose_time_ms = pose_frame.get_timestamp();
                        float dt_s = static_cast<float>(max(0.0, (now_ms - pose_time_ms) / 1000.0));
                        rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
                        pose[0] = predicted_pose.translation.x;
                        pose[1] = predicted_pose.translation.y;
                        pose[2] = predicted_pose.translation.z;
                        pose[3] = predicted_pose.rotation.x;
                        pose[4] = predicted_pose.rotation.y;
                        pose[5] = predicted_pose.rotation.z;
                        pose[6] = predicted_pose.rotation.w;
                    }
                    if (auto fs = frame.as<rs2::frameset>()) {
                        rs2::video_frame video_frame = frame.as<rs2::frameset>().get_fisheye_frame(1);//get the left frame
                        const int w = video_frame.get_width();
                        const int h = video_frame.get_height();
                        if (!hasReceivedCameraStream) {
                            rs2::stream_profile fisheye_stream = myProf.get_stream(RS2_STREAM_FISHEYE, 1);
                            intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics(); 
                            rs2_extrinsics pose_to_fisheye_extrinsics = myProf.get_stream(RS2_STREAM_POSE).get_extrinsics_to(fisheye_stream);
                            intrinsicsL = (cv::Mat_<double>(3, 3) << 
                                intrinsics.fx, 0, intrinsics.ppx,
                                0, intrinsics.fy, intrinsics.ppy, 0, 0, 1);
                            distCoeffsL = cv::Mat(1, 5, CV_32F, intrinsics.coeffs);
                            cv::Mat distortionCoefficients = (cv::Mat1d(4, 1) << distCoeffsL.at<double>(0, 0), distCoeffsL.at<double>(1, 0), distCoeffsL.at<double>(2, 0), distCoeffsL.at<double>(3, 0));
                            cv::Mat identity = cv::Mat::eye(3, 3, CV_64F);
                            cv::Mat PP1(3, 4, cv::DataType<float>::type);
                            intrinsicsL.copyTo(PP1.rowRange(0, 3).colRange(0, 3));
                            try {
                                cv::fisheye::initUndistortRectifyMap(intrinsicsL, distortionCoefficients, identity, PP1, cv::Size(848, 800), CV_32FC1, lm1, lm2);
                            }
                            catch (cv::Exception& e) {
                                Debug::Log(e.what(), Color::Red);
                            }

                        }
                        fisheye_mat = cv::Mat(cv::Size(w, h), CV_8UC1, (void*)video_frame.get_data(), cv::Mat::AUTO_STEP);         
                        cv::cvtColor(fisheye_mat, fisheye_mat_color, cv::COLOR_GRAY2BGRA);
                        if (!hasReceivedCameraStream) {
                            fisheye_mat_color_undistort = fisheye_mat_color.clone();
                        }
                        try {
                            cv::remap(fisheye_mat_color, fisheye_mat_color_undistort, lm1, lm2, cv::INTER_LINEAR);
                        }
                        catch (cv::Exception& e) {
                            Debug::Log(e.what(), Color::Red);
                        }
                        if (!hasReceivedCameraStream) {
                            textureChannels = 4;
                            if (textureInitializedCallback != nullptr) {
                                double fovX, fovY = 0;
                                double focalLength = 0;
                                double aspectRatio = 0;
                                cv::Point2d myPrincipalPoint;
                                cv::calibrationMatrixValues(intrinsicsL, cv::Size(848, 800), 0, 0, fovX, fovY, focalLength,myPrincipalPoint,aspectRatio);
                                textureInitializedCallback(w, h, 4, intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy,fovX,fovY,focalLength);
                            }
                            hasReceivedCameraStream = true;
                        }
                    }
                    });



                Debug::Log("Tracker started!", Color::Green);
                shouldRestart = false;

                while (!shouldRestart && !DoExit3) {
                    if (grabOriginPose) {
                        grabOriginPose = false;
                        if (tm_sensor.get_static_node("bugorigin", object_in_world_pose_frame.translation, object_in_world_pose_frame.rotation)) {
                            if (callbackObjectPoseReceived != nullptr) {
                                callbackObjectPoseReceived("origin_of_map", object_in_world_pose_frame.translation.x, object_in_world_pose_frame.translation.y, object_in_world_pose_frame.translation.z, object_in_world_pose_frame.rotation.x, object_in_world_pose_frame.rotation.y, object_in_world_pose_frame.rotation.z, object_in_world_pose_frame.rotation.w);
                            }
                            else {
                                Debug::Log("Wtf? There's no callback?", Color::Red);
                            }
                        }
                        else {
                            Debug::Log("ERROR: The object could not be found!", Color::Red);
                        }
                    }
                    //need to get callback saying we have images ready to go!

                    //Do we need to grab the current local map?
                    if (shouldGrabMap) {
                        shouldGrabMap = false;    
                        rs2_pose p;
                        p.translation.x = 0;
                        p.translation.y = 0;
                        p.translation.z = 0;
                        p.rotation.x = 0;
                        p.rotation.y = 0;
                        p.rotation.z = 0;
                        p.rotation.w = 1;
                        if (tm_sensor.set_static_node("bugorigin", p.translation, p.rotation)) {
                            ostringstream oss;
                            oss << "Exported pose for: " << "origin" << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                        }
                        else {
                            ostringstream oss;
                            oss << "Unable to export pose for: " << "origin" << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                        }
                        try {
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            rs2::calibration_table res = tm_sensor.export_localization_map();
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            raw_file_from_bytes("temp.raw", res);
                            callbackBinaryMap(NULL, NULL);
                        }
                        catch (const std::exception& ex) {
                            // ...
                            ostringstream oss;
                            oss << "Map save failed" << ex.what() << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                        }
                        catch (const std::string& ex) {
                            // ...
                            ostringstream oss;
                            oss << "Map save failed" << ex << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                        }
                        catch (...) {
                            ostringstream oss;
                            oss << "Map save failed: Unknown" << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                        }
                    }
                    //Do we need to load the map? if so exit all loops to allow the device to shutdown, setting the relevant flags
                    if (shouldLoadMap) {
                        shouldLoadMap = false;
                        shouldUploadData = true;
                        shouldRestart = true;
                    }
                    if (DoExit3) {//if we should exit, exit the tracker loop first
                        shouldRestart = true;
                    }
                }
                pipe.stop(); 
                if (usesIntegrator) {
                    if (Serial->WriteSerialPort("r")) {
                        Debug::Log("Restarted the t265/1",Color::Green);
                    }
                }
                if (DoExit3) {
                    break;
                }
            }
            catch (const rs2::error& e)
            {
                std::stringstream ss;
                ss << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
                mapDataLoaded = false;
                Debug::Log(ss.str(), Color::Red);
            }
            catch (const std::exception& e)
            {
                std::stringstream ss;
                std::cerr << e.what() << std::endl;
                mapDataLoaded = false;
                Debug::Log(ss.str(), Color::Red);
            }

        }
        if (usesIntegrator) {
            Serial->CloseSerialPort();
        }
        Debug::Log("Closed Tracker!",Color::Green);   
    }
};
