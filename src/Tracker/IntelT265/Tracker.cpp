#include <glm/common.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
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
#ifdef __linux__ //OGL

#else
#include <d3d11.h> //DX
#include "IUnityGraphicsD3D11.h"
#endif
#include "common_header.h"

#include "IUnityInterface.h"
#include "IUnityGraphics.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#ifdef __linux__
    #include <LinuxSerialPort.hpp>
    using namespace mn::CppLinuxSerial;
#else
    #include "Serial.h"
#endif
#define PI 3.141592653
typedef void(*FuncReceiveCameraImageCallbackWithID)(int instanceID, unsigned char* info, 
                                                    int lengthofarray, int width, int height, int pixelCount);


typedef void(*LocalizationCallback)(int trackerID, int LocalizationDelegate);
typedef void(*MapDataCallback)(int trackerID);
typedef void(*LocalizationPoseCallback)(int trackerID, string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
typedef void(*FuncDeltaMatrixConvertCallback)(float* matrixToReturn, bool isLeft, float tx_A, float ty_A, float tz_A, float qx_A, float qy_A, float qz_A, float qw_A, float tx_B, float ty_B, float tz_B, float qx_B, float qy_B, float qz_B, float qw_B);
typedef void(*QuaternionCallback)(float* arrayToCopy, float eux, float euy, float euz);
typedef void(*FuncDeltaPoseUpdateCallback)(int trackerID, float* poseDataLeft, float* poseDataRight);//needs refactoring into a map callback to subscribe to
typedef void(*FuncTextureInitializedCallback)(int trackerID, int TextureWidth, int TextureHeight, int textureCount, float fx, float fy, float cx, float cy, float fovx, float fovy, float focalLength, float d1, float d2, float d3, float d4, float d5);
class SensorInfoCallback {
public:
    FuncReceiveCameraImageCallbackWithID callbackWithID;
    int instanceID = 0;
};
class TrackerObject {
public:
    std::vector<SensorInfoCallback*> subscribedImageReceivers;
    rs2_intrinsics intrinsics;
    QuaternionCallback quaternionCallback = nullptr;
    FuncTextureInitializedCallback textureInitializedCallback = nullptr;
    LocalizationCallback callbackLocalization = nullptr;
    MapDataCallback callbackBinaryMap = nullptr;
    LocalizationPoseCallback callbackObjectPoseReceived = nullptr;
    FuncDeltaMatrixConvertCallback callbackMatrixConvert = nullptr;
    FuncDeltaPoseUpdateCallback callbackDeltaPoseUpdate = nullptr;
#ifdef __linux__ //OGL
#else
    ID3D11Device* m_Device;
    ID3D11Texture2D* d3dtex;
#endif
    int TrackerID;
    bool resetInitialPose = true;
    bool LockImage = false;

    float* latestPose = new float[7] {0, 0, 0, 0, 0, 0, 0};    
    float* deltaPoseLeftArray = new float[16]{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float* deltaPoseRightArray = new float[16]{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float fx, fy, cx, cy, d1, d2, d3, d4, d5 = 0;

    glm::mat4 leftEyeTransform;
    glm::mat4 rightEyeTransform;   
    glm::mat4 PoseInitial;
    glm::mat4 PoseFinal;

    glm::mat4 DeltaLeftEye, DeltaRightEye;

    cv::Mat fisheye_mat;
    cv::Mat fisheye_mat_color;
    cv::Mat fisheye_mat_color_undistort;
    cv::Mat fisheye_mat_color_cpu;
    cv::Mat lm1, lm2;

    rs2_pose object_in_world_pose_frame;
    bool hasLocalized = false;
    unsigned char* fileLocation;
    int textureWidth = 0;
    int textureHeight = 0;
    int textureChannels = 0;
    bool usesIntegrator = false;

    cv::Mat distCoeffsL;
    cv::Mat intrinsicsL; 
    bool hasReceivedCameraStream = false;
    char* com_port = "\\\\.\\COM5";
#ifdef __linux__

#else
    DWORD COM_BAUD_RATE = CBR_9600;
    SimpleSerial* Serial; 
#endif
    // Create a configuration for configuring the pipeline with a non default profile
    std::vector<unsigned char> myMapData;
    bool mapDataLoaded = false; 
    bool grabOriginPose = false;
    bool isFirst = true;
    bool ExitThreadLoop = false;
    bool shouldRestart = false;
    bool shouldLoadMap = false;
    bool shouldGrabMap = false;
    bool shouldUploadData = false;
    rs2::context myCon;
    rs2::device myDev;
    void ConvMatrixToFloatArray(glm::mat4 src, float* target) { 
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++){
                target[y * 4 + x] = src[x][y];
            }
        }
    }
    void FlagMapImport() {
       shouldLoadMap = true;
    }
    void SetComPortString(int port) {
        switch (port) {
#ifdef __linux__
        case 0:
            com_port = "/dev/ttyACM0";
            break;
        case 1:
            com_port = "/dev/ttyACM1";
            break;
        case 2:
            com_port = "/dev/ttyACM2";
            break;
        case 3:
            com_port = "/dev/ttyACM3";
            break;
        case 4:
            com_port = "/dev/ttyACM4";
            break;
        case 5:
            com_port = "/dev/ttyACM5";
            break;
        case 6:
            com_port = "/dev/ttyACM6";
            break;
        case 7:
            com_port = "/dev/ttyACM7";
            break;
        case 8:
            com_port = "/dev/ttyACM8";
            break;
#else
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
#endif
        }

    }
    void DoFunctionTracking() {
        if (usesIntegrator) {
#ifdef __linux__
            SerialPort serialPort(com_port, BaudRate::B_9600);
            serialPort.SetTimeout(-1); // Block when reading until any data is received
            serialPort.Open();
            serialPort.Write("r\r\n");
            serialPort.Close();
#else
            Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
            if (Serial->WriteSerialPort("r\r\n")) { 
                Debug::Log("Restarted the t265/1",Color::Green);  
                std::this_thread::sleep_for(std::chrono::seconds(1));
            } 
            Serial->CloseSerialPort();
#endif
        }
        while (!ExitThreadLoop) {
            try {
                bool receivedPoseFirst = false;                
                hasReceivedCameraStream = false;
                rs2::pipeline pipe;
                rs2::config cfg;
                rs2::pipeline_profile myProf;
                std::vector<std::string> serials;
                uint32_t dev_q;
                rs2::context ctx;
                dev_q = ctx.query_devices().size();
                cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
                cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE,1);
                cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE,2);
                for (rs2::device &&dev : ctx.query_devices())
                {
                    serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
                }
                cfg.enable_device(serials[TrackerID]);
                rs2::pose_sensor tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
                tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
                    if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                        hasLocalized = true;
                        if (callbackLocalization != nullptr) {
                            callbackLocalization(TrackerID,1);
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
                double lastPoseTime_ms = 0.0;
                //Then let's initialize the sensor and begin tracking
                myProf = pipe.start(cfg, [&](rs2::frame frame) {
                    if (auto fs = frame.as<rs2::pose_frame>()) {// For pose frames
                        rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>();
                        rs2_pose pose_data = pose_frame.get_pose_data();
                        auto now = std::chrono::system_clock::now().time_since_epoch();
                        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                        double pose_time_ms = pose_frame.get_timestamp();
                        float deltaTimeSinceLastPose = static_cast<float>(max(0.0, (now_ms - lastPoseTime_ms) / 1000.0));
                        float dt_s = static_cast<float>(max(0.0, (now_ms - pose_time_ms) / 1000.0));
                        rs2_pose predicted_pose = predict_pose(pose_data, dt_s);                        
                        latestPose[0] = predicted_pose.translation.x;
                        latestPose[1] = predicted_pose.translation.y;
                        latestPose[2] = -predicted_pose.translation.z; 
                        latestPose[3] = -predicted_pose.rotation.x;
                        latestPose[4] = -predicted_pose.rotation.y;
                        latestPose[5] = predicted_pose.rotation.z;
                        latestPose[6] = predicted_pose.rotation.w;
                        if (resetInitialPose) {
                            resetInitialPose = false;
                            PoseInitial = glm::toMat4(glm::qua<float>(predicted_pose.rotation.w, predicted_pose.rotation.y, -predicted_pose.rotation.x, predicted_pose.rotation.z));
                            PoseInitial[3][0] = predicted_pose.translation.y;
                            PoseInitial[3][1] = -predicted_pose.translation.x;
                            PoseInitial[3][2] = predicted_pose.translation.z;
                        }                     
                        PoseFinal = glm::toMat4(glm::qua<float>(predicted_pose.rotation.w, predicted_pose.rotation.y, -predicted_pose.rotation.x, predicted_pose.rotation.z));
                        PoseFinal[3][0] = predicted_pose.translation.y; 
                        PoseFinal[3][1] = -predicted_pose.translation.x; 
                        PoseFinal[3][2] = predicted_pose.translation.z;                          
                        try {
                            DeltaLeftEye = glm::inverse(leftEyeTransform) * glm::inverse(PoseInitial) * PoseFinal * leftEyeTransform; 
                            DeltaRightEye = glm::inverse(rightEyeTransform) * glm::inverse(PoseInitial) * PoseFinal * rightEyeTransform;                                                                         
                            ConvMatrixToFloatArray(DeltaLeftEye, deltaPoseLeftArray);
                            ConvMatrixToFloatArray(DeltaRightEye, deltaPoseRightArray);                         
                            if (callbackDeltaPoseUpdate != nullptr) {
                                callbackDeltaPoseUpdate(TrackerID, deltaPoseLeftArray, deltaPoseRightArray);
                            }
                        }
                        catch (std::exception e) {
                            Debug::Log(e.what(), Color::Red);
                        }
                        receivedPoseFirst = true;
                    }  
                    if (auto fs = frame.as<rs2::frameset>()) {// For camera frames
                        if (receivedPoseFirst) {
                            rs2::video_frame video_frame = frame.as<rs2::frameset>().get_fisheye_frame(1);
                            const int w = video_frame.get_width();
                            const int h = video_frame.get_height();
                            if (!hasReceivedCameraStream) {
                                rs2::stream_profile fisheye_stream = myProf.get_stream(RS2_STREAM_FISHEYE, 1);
                                intrinsics = fisheye_stream.as<rs2::video_stream_profile>().get_intrinsics();
                                rs2_extrinsics pose_to_fisheye_extrinsics = myProf.get_stream(RS2_STREAM_POSE).get_extrinsics_to(fisheye_stream);
                                fx = intrinsics.fx;
                                fy = intrinsics.fy;
                                cx = intrinsics.ppx;
                                cy = intrinsics.ppy;
                                d1 = intrinsics.coeffs[0];
                                d2 = intrinsics.coeffs[1];
                                d3 = intrinsics.coeffs[2];
                                d4 = intrinsics.coeffs[3];                                
                                intrinsicsL = (cv::Mat_<double>(3, 3) <<
                                    fx, 0, cx,
                                    0, fy, cy, 0, 0, 1);
                                distCoeffsL = cv::Mat(1, 5, CV_32F, intrinsics.coeffs);
                                cv::Mat distortionCoefficients = (cv::Mat1d(4, 1) << intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2], intrinsics.coeffs[3]);
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
                            cv::cvtColor(fisheye_mat, fisheye_mat_color, cv::COLOR_GRAY2BGRA,4);
                            if (!hasReceivedCameraStream) {
                                fisheye_mat_color_undistort = fisheye_mat_color.clone();
                                fisheye_mat_color_cpu = fisheye_mat_color.clone();
                            }
                            try {
                                if (!LockImage) {
                                    cv::remap(fisheye_mat_color, fisheye_mat_color_undistort, lm1, lm2, cv::INTER_LINEAR);
                                    fisheye_mat_color_cpu = fisheye_mat_color_undistort.clone();
                                    for (std::vector<SensorInfoCallback*>::iterator it = subscribedImageReceivers.begin(); it != subscribedImageReceivers.end(); ++it) {
                                        if ((*it) != nullptr) {
                                            if ((*it)->callbackWithID != nullptr) {
                                                (*it)->callbackWithID((*it)->instanceID, fisheye_mat_color_cpu.data, fisheye_mat_color_cpu.rows * fisheye_mat_color_cpu.cols, fisheye_mat_color_cpu.cols, fisheye_mat_color_cpu.rows, fisheye_mat_color_cpu.channels());
                                            }
                                        }
                                    }
                                }
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
                                    cv::calibrationMatrixValues(intrinsicsL, cv::Size(w, h), 0, 0, fovX, fovY, focalLength, myPrincipalPoint, aspectRatio);
                                    textureInitializedCallback(TrackerID,w, h, 4, fx,fy,cx,cy, fovX, fovY, focalLength, d1,d2, d3, d4, 1);
                                }
                                hasReceivedCameraStream = true;
                            }

                            
                        }
                    }
                    });                
                shouldRestart = false;
                while (!shouldRestart && !ExitThreadLoop) {
                    if (grabOriginPose) {
                        grabOriginPose = false;
                        if (tm_sensor.get_static_node("bugorigin", object_in_world_pose_frame.translation, object_in_world_pose_frame.rotation)) {
                            if (callbackObjectPoseReceived != nullptr) {
                                callbackObjectPoseReceived(TrackerID,"origin_of_map", object_in_world_pose_frame.translation.x, object_in_world_pose_frame.translation.y, object_in_world_pose_frame.translation.z, object_in_world_pose_frame.rotation.x, object_in_world_pose_frame.rotation.y, object_in_world_pose_frame.rotation.z, object_in_world_pose_frame.rotation.w);
                            }
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
                        }
                        else {
                        }
                        try {
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            rs2::calibration_table res = tm_sensor.export_localization_map();
                            std::this_thread::sleep_for(std::chrono::seconds(2));
                            raw_file_from_bytes("temp.raw", res);
                            callbackBinaryMap(TrackerID);
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

                    if (ExitThreadLoop) {//if we should exit, exit the tracker loop first
                        shouldRestart = true;
                    }
                }
                if (usesIntegrator) {
#ifdef __linux__
                    SerialPort serialPort(com_port, BaudRate::B_9600); 
                    serialPort.SetTimeout(-1); // Block when reading until any data is received
                    serialPort.Open();
                    serialPort.Write("r\r\n");
                    serialPort.Close();
#else
                    Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
                    if (Serial->WriteSerialPort("r\r\n")) {
                        Debug::Log("Restarted the t265/1", Color::Green);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                    Serial->CloseSerialPort();
#endif 
                }
                pipe.stop(); 
                if (ExitThreadLoop) {
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
#ifdef __linux__
#else
        if (usesIntegrator) {
            Serial->CloseSerialPort();
        }
#endif
        Debug::Log("Closed Tracker!",Color::Green);   
    }
    void ResetInitialPose() {
        resetInitialPose = true;
    }
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
    void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
    {
        std::ofstream file(filename, std::ios::binary | std::ios::trunc);
        if (!file.good()) {
            Debug::Log("Invalid binary file specified. Verify the target path and location permissions");
        }
        file.write((char*)bytes.data(), bytes.size());
    }
    void UpdatecameraTextureGPU() {
#ifdef __linux__ //OGL 

#else
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
#endif
    }
    void StopTracking() {
        ExitThreadLoop = true;
    }
    void GrabPoseInOrigin() {
        if (hasLocalized) {
            grabOriginPose = true;
        }
        else {
            Debug::Log("ERROR: The map hasn't localized yet!", Color::Red);
        }
    }
    void SetTexturePointer(void* textureHandle) {
#ifdef __linux__ //OGL

#else //DX
        d3dtex = (ID3D11Texture2D*)textureHandle;
#endif
    }
    void GrabMap() {
        if (callbackBinaryMap != nullptr) {
            shouldGrabMap = true;
        }
    }
    void SubscribeReceiver(FuncReceiveCameraImageCallbackWithID callback, int callbackID) {
        SensorInfoCallback* c = new SensorInfoCallback();
        c->instanceID = callbackID;
        c->callbackWithID = callback;
        subscribedImageReceivers.push_back(c);
    }
    void SetLeftRightEyeTransforms(float* leftEyeTransform, float* rightEyeTransform) { 
        Debug::Log("Set my left and right eye transforms from origin", Color::Green);
        this->leftEyeTransform = {
            {leftEyeTransform[0],leftEyeTransform[1],leftEyeTransform[2],leftEyeTransform[3]},
            {leftEyeTransform[4],leftEyeTransform[5],leftEyeTransform[6],leftEyeTransform[7]},
            {leftEyeTransform[8],leftEyeTransform[9],leftEyeTransform[10],leftEyeTransform[11]},
            {leftEyeTransform[12],leftEyeTransform[13],leftEyeTransform[14],leftEyeTransform[15]},
        };
        this->rightEyeTransform = {
            {rightEyeTransform[0],rightEyeTransform[1],rightEyeTransform[2],rightEyeTransform[3]},
            {rightEyeTransform[4],rightEyeTransform[5],rightEyeTransform[6],rightEyeTransform[7]},
            {rightEyeTransform[8],rightEyeTransform[9],rightEyeTransform[10],rightEyeTransform[11]},
            {rightEyeTransform[12],rightEyeTransform[13],rightEyeTransform[14],rightEyeTransform[15]},
        };
        this->rightEyeTransform = transpose(this->rightEyeTransform); 
        this->leftEyeTransform = transpose(this->leftEyeTransform);
    }

};
