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
#include <chrono>
#include <thread>
#include <mutex>
#include <math.h>
#include <stack>
#include <float.h>
#ifdef __linux__ //OGL

#else
#include <d3d11.h> //DX
#include "IUnityGraphicsD3D11.h"
#endif
#include "common_header.h"
#include "IUnityInterface.h"
#include "IUnityGraphics.h"
#ifdef __linux__
#include <LinuxSerialPort.hpp>
    using namespace mn::CppLinuxSerial;
#else
#include "Serial.h"
#endif
#include <DollaryDooFilter.h>
#include <KalmanFilter.h>
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
    TrackerObject() {
        Debug::Log("Setting Tracker ID");
        poseFilterK = KalmanFilterPose();
        slamFilterK = KalmanFilterPose();
        velocityFilterK = KalmanFilterPose();
        accellerationFilterK = KalmanFilterPose();
        angularVelocityFilterK = KalmanFilterPose();
        angularAccellerationFilterK = KalmanFilterPose();
        slamFilterDollaryDoo = OneDollaryDooFilterPose(200);
        velocityFilterDollaryDoo = OneDollaryDooFilterPose(200);
        accellerationFilterDollaryDoo = OneDollaryDooFilterPose(200);
        angularVelocityFilterDollaryDoo = OneDollaryDooFilterPose(200);
        angularAccellerationFilterDoo = OneDollaryDooFilterPose(200);
    }
    bool useKalmanFilterPose = false;


    KalmanFilterPose poseFilterK;
    OneDollaryDooFilterPose poseFilter;

    KalmanFilterPose slamFilterK;
    KalmanFilterPose velocityFilterK;
    KalmanFilterPose accellerationFilterK;

    KalmanFilterPose angularVelocityFilterK;
    KalmanFilterPose angularAccellerationFilterK;

    OneDollaryDooFilterPose slamFilterDollaryDoo;

    OneDollaryDooFilterPose velocityFilterDollaryDoo;
    OneDollaryDooFilterPose accellerationFilterDollaryDoo;

    OneDollaryDooFilterPose angularVelocityFilterDollaryDoo;
    OneDollaryDooFilterPose angularAccellerationFilterDoo;

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
    bool LockPose = true;

    double lastProvidedPoseTimestampMS = 0;
    double lastProvidedDeltaPoseTimestampMS = 0;

    double* latestDeltaPoseExternal = new double[7]{ 0,0,0,0,0,0,1 };
    double* latestPoseExternal = new double[7]{ 0,0,0,0,0,0,1 };
    double* latestDeltaPoseInternal = new double[7]{ 0,0,0,0,0,0,1 };
    double* latestPoseInternal = new double[7]{ 0,0,0,0,0,0,1 };


    double* latestTransExternal = new double[3]{ 0, 0, 0 };
    double* latestAccelExternal = new double[3]{ 0, 0, 0 };
    double* latestVeloExternal = new double[3]{ 0, 0, 0 };

    double* latestRotExternal = new double[4]{ 0, 0, 0, 1 };
    double* latestAngularAccelExternal = new double[3]{ 0, 0, 0 };
    double* latestAngularVeloExternal = new double[3]{ 0, 0, 0 };



    float* latestPose = new float[7]{ 0, 0, 0, 0, 0, 0, 1 };

    float* latestAccel = new float[3]{ 0, 0, 0 };
    float* latestVelo = new float[3]{ 0, 0, 0 };
    float* latestTrans = new float[3]{ 0, 0, 0 };

    float* latestAngularAccel = new float[3]{ 0, 0, 0 };
    float* latestAngularVelo = new float[3]{ 0, 0, 0 };
    float* latestRot = new float[4]{ 0, 0, 0, 1 };

    double lastPoseTimeStamp = 0;


    float* deltaPoseLeftArray = new float[16]{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float* deltaPoseRightArray = new float[16]{ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 };
    float fx, fy, cx, cy, d1, d2, d3, d4, d5 = 0;
    glm::mat4 leftEyeTransform;
    glm::mat4 rightEyeTransform;
    glm::mat4 PoseInitial;
    glm::mat4 PoseFinal;

    glm::mat4 DeltaLeftEye, DeltaRightEye;

   

    bool hasLocalized = false;
    unsigned char* fileLocation;
    int textureWidth = 0;
    int textureHeight = 0;
    int textureChannels = 0;
    bool usesIntegrator = false;
    float CurrentTimeOffset = 0;
    int trackingSystem = 0; //0 for old, 1 for new
    bool hasReceivedCameraStream = false;
    char* com_port = "\\\\.\\COM5";
#ifdef __linux__

#else
    DWORD COM_BAUD_RATE = CBR_9600;
    SimpleSerial* Serial;
#endif
    // Create a configuration for configuring the pipeline with a non default profile
    std::vector<unsigned char> myMapData;

    bool receivedPoseFirst = false;
    bool mapDataLoaded = false;
    bool grabOriginPose = false;
    bool isFirst = true;
    bool ExitThreadLoop = false;
    bool usingFrame = false;
    bool shouldRestart = false;
    bool shouldLoadMap = false;
    bool shouldGrabMap = false;
    bool shouldUploadData = false;
    bool hasReceivedTexture = false;
    bool initializeWithPassthrough = false;

    bool filterEnabled = true;
    bool kFilterEnabled = true;
    float rfreq, rmincutoff, rbeta, rdcutoff = 0;
    float tfreq, tmincutoff, tbeta, tdcutoff = 0;
    float qt, rt = 1.0;
    float qr, rr = 0.0;
    void ConvMatrixToFloatArray(glm::mat4 src, float* target) {
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                target[y * 4 + x] = src[x][y];
            }
        }
    }
    void FlagMapImport() {
        shouldLoadMap = true;
    }
    void UpdateTimeOffset(float time) {
        CurrentTimeOffset = time;
    }
    void UpdateFilterTranslationParams(double _freq, double _mincutoff, double _beta, double _dcutoff) {
        tfreq = _freq;
        tmincutoff = _mincutoff;
        tbeta = _beta;
        tdcutoff = _dcutoff;
        poseFilter.UpdateTranslationParams(_freq, _mincutoff, _beta, _dcutoff);
    }
    void UpdateKFilterTranslationParams(float _qt, float _rt) {
        qt = _qt;
        rt = _rt;
        poseFilterK.UpdateTranslationParams(qt, rt);
        poseFilterK.Reset();
    }
    void UpdateKFilterRotationParams(float _qr, float _rr) {
        qr = _qr;
        rr = _rr;
        poseFilterK.UpdateRotationParams(qr, rr);
        poseFilterK.Reset();
    }

    void SetFilterEnabled(bool value) {
        filterEnabled = value;
        poseFilter.SetFilterEnabled(value);
    }
    void SetKFilterEnabled(bool value) {
        kFilterEnabled = value;
        poseFilterK.SetFilterEnabled(value);
    }
    void SetFilterEnabledExt(bool slam, bool velocity, bool accel, bool angvelocity, bool angaccel) {
        slamFilterDollaryDoo.SetFilterEnabled(slam);
        velocityFilterDollaryDoo.SetFilterEnabled(velocity);
        accellerationFilterDollaryDoo.SetFilterEnabled(accel);
        angularVelocityFilterDollaryDoo.SetFilterEnabled(angvelocity);
        angularAccellerationFilterDoo.SetFilterEnabled(angaccel);
    }
    void SetKFilterEnabledExt(bool slam, bool velocity, bool accel, bool angvelocity, bool angaccel) {
        slamFilterK.SetFilterEnabled(slam);
        velocityFilterK.SetFilterEnabled(velocity);
        accellerationFilterK.SetFilterEnabled(accel);
        angularVelocityFilterK.SetFilterEnabled(angvelocity);
        angularAccellerationFilterK.SetFilterEnabled(angaccel);
    }
    void ResetFilters() {
        poseFilter.Reset();
        poseFilterK.Reset();
    }
    void ResetFiltersExt() {
        slamFilterK.Reset();
        velocityFilterK.Reset();
        accellerationFilterK.Reset();
        angularVelocityFilterK.Reset();
        angularAccellerationFilterK.Reset();
    }
    void UpdateFilterRotationParams(double _freq, double _mincutoff, double _beta, double _dcutoff) {
        rfreq = _freq;
        rmincutoff = _mincutoff;
        rbeta = _beta;
        rdcutoff = _dcutoff;
        poseFilter.UpdateRotationParams(_freq, _mincutoff, _beta, _dcutoff);
    }
    void UpdateTransFilterDollaryDooParams(double _transfreq, double _transmincutoff, double _transbeta, double _transdcutoff,
        double _velfreq, double _velmincutoff, double _velbeta, double _veldcutoff,
        double _accelfreq, double _accelmincutoff, double _accelbeta, double _acceldcutoff
    ) {
        slamFilterDollaryDoo.UpdateTranslationParams(_transfreq, _transmincutoff, _transbeta, _transdcutoff);
        velocityFilterDollaryDoo.UpdateTranslationParams(_velfreq, _velmincutoff, _velbeta, _veldcutoff);
        accellerationFilterDollaryDoo.UpdateTranslationParams(_accelfreq, _accelmincutoff, _accelbeta, _acceldcutoff);
        slamFilterDollaryDoo.Reset();
        velocityFilterDollaryDoo.Reset();
        accellerationFilterDollaryDoo.Reset();
    }
    void UpdateRotFilterDollaryDooParams(double _rotfreq, double _rotmincutoff, double _rotbeta, double _rotdcutoff,
        double _velfreq, double _velmincutoff, double _velbeta, double _veldcutoff,
        double _accelfreq, double _accelmincutoff, double _accelbeta, double _acceldcutoff
    ) { 
        slamFilterDollaryDoo.UpdateRotationParams(_rotfreq, _rotmincutoff, _rotbeta, _rotdcutoff);
        angularVelocityFilterDollaryDoo.UpdateTranslationParams(_velfreq, _velmincutoff, _velbeta, _veldcutoff);
        angularAccellerationFilterDoo.UpdateTranslationParams(_accelfreq, _accelmincutoff, _accelbeta, _acceldcutoff);
        slamFilterDollaryDoo.Reset();
        angularVelocityFilterDollaryDoo.Reset();
        angularAccellerationFilterDoo.Reset();
    }
    void UpdateTransFilterKParams(double _transq, double _transr,
        double _velq, double _velr,
        double _accelq, double _accelr
    ) {
        slamFilterK.UpdateTranslationParams(_transq, _transr);
        velocityFilterK.UpdateTranslationParams(_transq, _transr);
        accellerationFilterK.UpdateTranslationParams(_transq, _transr);
        slamFilterK.Reset();
        velocityFilterK.Reset();
        accellerationFilterK.Reset();
    }
    void UpdateRotFilterKParams(double _rotq, double _rotr,
        double _angvelq, double _angvelr,
        double _angaccelq, double _angaccelr
    ) {
        slamFilterK.UpdateRotationParams(_rotq, _rotr);
        angularVelocityFilterK.UpdateTranslationParams(_angvelq, _angvelr);
        angularAccellerationFilterK.UpdateTranslationParams(_angaccelq, _angaccelr);
        slamFilterK.Reset();
        angularVelocityFilterK.Reset();
        angularAccellerationFilterK.Reset();
    }

    double* GetLatestTimestampPose() {//in theory, by extrapolating the sensor values we should get the pose at the lateset time stamp, can't be arsed translating this to the shared math library, will use intels for now
        auto now = std::chrono::system_clock::now().time_since_epoch(); 
        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        rs2_pose pose;
        rs2_pose predicted_pose;
        float dt_s = static_cast<float>(max(0.0, (now_ms - lastPoseTimeStamp) / 1000.0));

        pose.translation.x = latestTransExternal[0]; 
        pose.translation.y = latestTransExternal[1]; 
        pose.translation.z = latestTransExternal[2];
        pose.rotation.x = latestRotExternal[0]; 
        pose.rotation.y = latestRotExternal[1]; 
        pose.rotation.z = latestRotExternal[2]; 
        pose.rotation.w = latestRotExternal[3];

        pose.acceleration.x = latestAccelExternal[0]; 
        pose.acceleration.y = latestAccelExternal[1]; 
        pose.acceleration.z = latestAccelExternal[2];
        pose.velocity.x = latestVeloExternal[0]; 
        pose.velocity.y = latestVeloExternal[1]; 
        pose.velocity.z = latestVeloExternal[2];
        
        pose.angular_acceleration.x = latestAngularAccelExternal[0]; 
        pose.angular_acceleration.y = latestAngularAccelExternal[1]; 
        pose.angular_acceleration.z = latestAngularAccelExternal[2];
        pose.angular_velocity.x = latestAngularVeloExternal[0]; 
        pose.angular_velocity.y = latestAngularVeloExternal[1]; 
        pose.angular_velocity.z = latestAngularVeloExternal[2];
        
        predicted_pose.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
        predicted_pose.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
        predicted_pose.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
        rs2_vector W = {
                dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
                dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
                dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
        };
        predicted_pose.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
        latestPoseExternal[0] = predicted_pose.translation.x; latestPoseExternal[1] = predicted_pose.translation.y; latestPoseExternal[2] = predicted_pose.translation.z;
        latestPoseExternal[3] = predicted_pose.rotation.x; latestPoseExternal[4] = predicted_pose.rotation.y; latestPoseExternal[5] = predicted_pose.rotation.z; latestPoseExternal[6] = predicted_pose.rotation.w;
        return latestPoseExternal;
    }
    double* GetOffsetTimestampPose(double msOffset) {//in theory, by extrapolating the sensor values we should get the pose at the lateset time stamp, can't be arsed translating this to the shared math library, will use intels for now
        auto now = std::chrono::system_clock::now().time_since_epoch();
        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
        rs2_pose pose;
        rs2_pose predicted_pose;
        float dt_s = static_cast<float>(max(0.0, (now_ms - lastPoseTimeStamp) / 1000.0) + msOffset);

        pose.translation.x = latestTransExternal[0]; pose.translation.y = latestTransExternal[1]; pose.translation.z = latestTransExternal[2];
        pose.rotation.x = latestRotExternal[0]; pose.rotation.y = latestRotExternal[1]; pose.rotation.z = latestRotExternal[2]; pose.rotation.w = latestRotExternal[3];

        pose.acceleration.x = latestAccelExternal[0]; pose.acceleration.y = latestAccelExternal[1]; pose.acceleration.z = latestAccelExternal[2];
        pose.velocity.x = latestVeloExternal[0]; pose.velocity.y = latestVeloExternal[1]; pose.velocity.z = latestVeloExternal[2];
        pose.translation.x = latestTransExternal[0]; pose.translation.y = latestTransExternal[1];  pose.translation.z = latestTransExternal[2];
        pose.angular_acceleration.x = latestAngularAccelExternal[0]; pose.angular_acceleration.y = latestAngularAccelExternal[1]; pose.angular_acceleration.z = latestAngularAccelExternal[2];
        pose.angular_velocity.x = latestAngularVeloExternal[0]; pose.angular_velocity.y = latestAngularVeloExternal[1]; pose.angular_velocity.z = latestAngularVeloExternal[2];
        predicted_pose.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
        predicted_pose.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
        predicted_pose.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
        rs2_vector W = {
                dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
                dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
                dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
        };
        predicted_pose.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
        latestPoseExternal[0] = predicted_pose.translation.x; latestPoseExternal[1] = predicted_pose.translation.y; latestPoseExternal[2] = predicted_pose.translation.z;
        latestPoseExternal[3] = predicted_pose.rotation.x; latestPoseExternal[4] = predicted_pose.rotation.y; latestPoseExternal[5] = predicted_pose.rotation.z; latestPoseExternal[6] = predicted_pose.rotation.w;
        return latestPoseExternal;
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
    double last_ms = 0;
    void DoFunctionTracking() { // This is the main tracking loop initiated by the system
        if (usesIntegrator) {//this is used to flag a disconnect/reconnect of the t261 to fix it's 'locking' issue, unused by the X sensor
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
        double tempx, tempy, tempz = 0;
        double tempw = 1;
        while (!ExitThreadLoop) { //start a loop for when we reset the device
            try {
                Debug::Log("Starting Thread", Color::Green);
                // Initialize pose filters
                poseFilter = OneDollaryDooFilterPose(tfreq, tmincutoff, tbeta, tdcutoff);
                

                poseFilter.UpdateTranslationParams(tfreq, tmincutoff, tbeta, tdcutoff);
                poseFilter.UpdateRotationParams(rfreq, rmincutoff, rbeta, rdcutoff);
                poseFilter.SetFilterEnabled(filterEnabled);
                poseFilterK.SetDefault(qt, rt, qr, rr);

                poseFilterK.SetFilterEnabled(filterEnabled);


                hasReceivedCameraStream = false;
                receivedPoseFirst = false;
                rs2::pipeline pipe;
                rs2::config cfg;
                rs2::pipeline_profile myProf;
                std::vector<std::string> serials;
                uint32_t dev_q;
                rs2::context ctx;
                cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);//We prevent the use of the see through to ensure edge compute mode
                // If we're using pass through, enable the streams
/*                if (initializeWithPassthrough) {
                    cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE, 1);
                    cfg.enable_stream(rs2_stream::RS2_STREAM_FISHEYE, 2);
                }*/
                rs2::pose_sensor tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
                tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
                    // This callback handles all of the sensor's code
                    if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                        hasLocalized = true;
                        // The LLAPI automatically sets up these pointer functions, on device relocalization call them
                        if (callbackLocalization != nullptr) {
                            callbackLocalization(TrackerID, 1);
                        }
                        else {
                            Debug::Log("The callback doesn't exist, wtf?", Color::Red);
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

                double last_msps = 0;
                //Then let's initialize the sensor and begin tracking 
                myProf = pipe.start(cfg, [&](rs2::frame frame) {
                    bool hasPose = false;
                    bool hasImage = false;
                    if (rs2::pose_frame fs = frame.as<rs2::pose_frame>()) {// For pose frames
                        hasPose = true;
                        LockPose = true;
                        rs2_pose pose = fs.get_pose_data();
                        rs2_pose predicted_pose = pose;
                        lastPoseTimeStamp = fs.get_timestamp();

                        latestTrans[0] = pose.translation.x; latestTrans[1] = pose.translation.y; latestTrans[2] = pose.translation.z;
                        latestRot[0] = pose.rotation.x; latestRot[1] = pose.rotation.y; latestRot[2] = pose.rotation.z; latestRot[3] = pose.rotation.w;

                        latestAccel[0] = pose.acceleration.x; latestAccel[1] = pose.acceleration.y; latestAccel[2] = pose.acceleration.z;
                        latestVelo[0] = pose.velocity.x; latestVelo[1] = pose.velocity.y; latestVelo[2] = pose.velocity.z;


                        latestAngularAccel[0] = pose.angular_acceleration.x; latestAngularAccel[1] = pose.angular_acceleration.y; latestAngularAccel[2] = pose.angular_acceleration.z;
                        latestAngularVelo[0] = pose.angular_velocity.x; latestAngularVelo[1] = pose.angular_velocity.y; latestAngularVelo[2] = pose.angular_velocity.z;                        

                        auto now = std::chrono::system_clock::now().time_since_epoch();
                        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                        lastProvidedPoseTimestampMS = now_ms;

                        receivedPoseFirst = true;
                        LockPose = false;

                        float dt_p = static_cast<float>(max(0.0, (now_ms - last_msps) / 1000.0));
                        float dt_s = static_cast<float>(max(0.0, (now_ms - lastPoseTimeStamp) / 1000.0) + CurrentTimeOffset);

                        // The following does a per-segment filter on each part, exposing it via getlatestposetimestamp
                        /**/
                        slamFilterK.Filter(latestTrans[0], latestTrans[1], latestTrans[2], latestRot[0], latestRot[1], latestRot[2], latestRot[3]);
                        velocityFilterK.Filter(latestVelo[0], latestVelo[1], latestVelo[2]);
                        accellerationFilterK.Filter(latestAccel[0], latestAccel[1], latestAccel[2]);
                        angularVelocityFilterK.Filter(latestAngularAccel[0], latestAngularAccel[1], latestAngularAccel[2]);
                        angularAccellerationFilterK.Filter(latestAngularVelo[0], latestAngularVelo[1], latestAngularVelo[2]);

                        slamFilterK.ObtainFilteredPose(latestTransExternal[0], latestTransExternal[1], latestTransExternal[2], latestRotExternal[0], latestRotExternal[1], latestRotExternal[2], latestRotExternal[3]);
                        velocityFilterK.ObtainFilteredPose(latestVeloExternal[0], latestVeloExternal[1], latestVeloExternal[2]);
                        accellerationFilterK.ObtainFilteredPose(latestAccelExternal[0], latestAccelExternal[1], latestAccelExternal[2]);
                        angularVelocityFilterK.ObtainFilteredPose(latestAngularVeloExternal[0], latestAngularVeloExternal[1], latestAngularVeloExternal[2]);
                        angularAccellerationFilterK.ObtainFilteredPose(latestAngularAccelExternal[0], latestAngularAccelExternal[1], latestAngularAccelExternal[2]);

                        slamFilterDollaryDoo.Filter(latestTransExternal[0], latestTransExternal[1], latestTransExternal[2], latestRotExternal[0], latestRotExternal[1], latestRotExternal[2], latestRotExternal[3], dt_s);
                        velocityFilterDollaryDoo.Filter(latestVeloExternal[0], latestVeloExternal[1], latestVeloExternal[2], dt_s);
                        accellerationFilterDollaryDoo.Filter(latestAccelExternal[0], latestAccelExternal[1], latestAccelExternal[2], dt_s);
                        angularVelocityFilterDollaryDoo.Filter(latestAngularVeloExternal[0], latestAngularVeloExternal[1], latestAngularVeloExternal[2], dt_s);
                        angularAccellerationFilterDoo.Filter(latestAngularAccelExternal[0], latestAngularAccelExternal[1], latestAngularAccelExternal[2], dt_s);

                        slamFilterDollaryDoo.ObtainFilteredPose(latestTransExternal[0], latestTransExternal[1], latestTransExternal[2], latestRotExternal[0], latestRotExternal[1], latestRotExternal[2], latestRotExternal[3]);
                        velocityFilterDollaryDoo.ObtainFilteredPose(latestVeloExternal[0], latestVeloExternal[1], latestVeloExternal[2]);
                        accellerationFilterDollaryDoo.ObtainFilteredPose(latestAccelExternal[0], latestAccelExternal[1], latestAccelExternal[2]);
                        angularVelocityFilterDollaryDoo.ObtainFilteredPose(latestAngularVeloExternal[0], latestAngularVeloExternal[1], latestAngularVeloExternal[2]);
                        angularAccellerationFilterDoo.ObtainFilteredPose(latestAngularAccelExternal[0], latestAngularAccelExternal[1], latestAngularAccelExternal[2]);

                        if (trackingSystem == 0) {
                            pose.translation.x = latestTrans[0]; pose.translation.y = latestTrans[1];  pose.translation.z = latestTrans[2];
                            pose.rotation.x = latestRot[0]; pose.rotation.y = latestRot[1]; pose.rotation.z = latestRot[2]; pose.rotation.w = latestRot[3];

                            pose.acceleration.x = latestAccel[0]; pose.acceleration.y = latestAccel[1]; pose.acceleration.z = latestAccel[2];
                            pose.velocity.x = latestVelo[0]; pose.velocity.y = latestVelo[1]; pose.velocity.z = latestVelo[2];

                            pose.angular_acceleration.x = latestAngularAccel[0]; pose.angular_acceleration.y = latestAngularAccel[1]; pose.angular_acceleration.z = latestAngularAccel[2];
                            pose.angular_velocity.x = latestAngularVelo[0]; pose.angular_velocity.y = latestAngularVelo[1]; pose.angular_velocity.z = latestAngularVelo[2];

                            predicted_pose.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
                            predicted_pose.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
                            predicted_pose.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
                            rs2_vector W = {
                                    dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
                                    dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
                                    dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
                            };
                            predicted_pose.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
                            poseFilterK.Filter(predicted_pose.translation.x, predicted_pose.translation.y, -predicted_pose.translation.z, -predicted_pose.rotation.x, -predicted_pose.rotation.y, predicted_pose.rotation.z, predicted_pose.rotation.w);
                            poseFilterK.ObtainFilteredPose(latestPose[0], latestPose[1], latestPose[2], latestPose[3], latestPose[4], latestPose[5], latestPose[6]);
                            poseFilter.Filter(latestPose[0], latestPose[1], latestPose[2], latestPose[3], latestPose[4], latestPose[5], latestPose[6], dt_p);
                            poseFilter.ObtainFilteredPose(latestPose[0], latestPose[1], latestPose[2], latestPose[3], latestPose[4], latestPose[5], latestPose[6]);
                            //  This 'delta pose' calculates the last captured and rendered frame, and the current pose
                            if (resetInitialPose) {
                                //                    Debug::Log("Reset the initial pose", Color::Green);
                                resetInitialPose = false;
                                PoseInitial = glm::toMat4(glm::qua<float>(latestPose[6], latestPose[3], latestPose[4], latestPose[5]));

                                PoseInitial[3][0] = latestPose[0];
                                PoseInitial[3][1] = latestPose[1];
                                PoseInitial[3][2] = -latestPose[2];
                            }
                            PoseFinal = glm::toMat4(glm::qua<float>(latestPose[6], latestPose[3], latestPose[4], latestPose[5]));
                            PoseFinal[3][0] = latestPose[0];
                            PoseFinal[3][1] = latestPose[1];
                            PoseFinal[3][2] = -latestPose[2];
                            ostringstream oss;
                            oss << "Pose: " << latestPose[6] << "," << latestPose[3] << "," << latestPose[4] << "," << latestPose[5] << std::endl;
                            Debug::Log(oss.str(), Color::Green);
                            try {
                                DeltaLeftEye = glm::inverse(leftEyeTransform) * glm::inverse(PoseInitial) * PoseFinal * leftEyeTransform;
                                DeltaRightEye = glm::inverse(rightEyeTransform) * glm::inverse(PoseInitial) * PoseFinal * rightEyeTransform;
                                ConvMatrixToFloatArray(DeltaLeftEye, deltaPoseLeftArray);
                                ConvMatrixToFloatArray(DeltaRightEye, deltaPoseRightArray);
                                //  Once delta pose is calculated, call the function pointer callback
                                if (callbackDeltaPoseUpdate != nullptr) {
                                    callbackDeltaPoseUpdate(TrackerID, deltaPoseLeftArray, deltaPoseRightArray);
                                }
                            }
                            catch (std::exception e) {
                                Debug::Log(e.what(), Color::Red);
                            }
                        }
                        else {
                            // this is the old non filtered approach to receiving the noisy data from the t261
                            pose.translation.x = latestTransExternal[0]; pose.translation.y = latestTransExternal[1];  pose.translation.z = latestTransExternal[2];
                            pose.rotation.x = latestRotExternal[0]; pose.rotation.y = latestRotExternal[1]; pose.rotation.z = latestRotExternal[2]; pose.rotation.w = latestRotExternal[3];

                            pose.acceleration.x = latestAccelExternal[0]; pose.acceleration.y = latestAccelExternal[1]; pose.acceleration.z = latestAccelExternal[2];
                            pose.velocity.x = latestVeloExternal[0]; pose.velocity.y = latestVeloExternal[1]; pose.velocity.z = latestVeloExternal[2];

                            pose.angular_acceleration.x = latestAngularAccelExternal[0]; pose.angular_acceleration.y = latestAngularAccelExternal[1]; pose.angular_acceleration.z = latestAngularAccelExternal[2];
                            pose.angular_velocity.x = latestAngularVeloExternal[0]; pose.angular_velocity.y = latestAngularVeloExternal[1]; pose.angular_velocity.z = latestAngularVeloExternal[2];
 
                            predicted_pose.translation.x = dt_s * (dt_s / 2 * pose.acceleration.x + pose.velocity.x) + pose.translation.x;
                            predicted_pose.translation.y = dt_s * (dt_s / 2 * pose.acceleration.y + pose.velocity.y) + pose.translation.y;
                            predicted_pose.translation.z = dt_s * (dt_s / 2 * pose.acceleration.z + pose.velocity.z) + pose.translation.z;
                            rs2_vector W = {
                                   dt_s * (dt_s / 2 * pose.angular_acceleration.x + pose.angular_velocity.x),
                                   dt_s * (dt_s / 2 * pose.angular_acceleration.y + pose.angular_velocity.y),
                                   dt_s * (dt_s / 2 * pose.angular_acceleration.z + pose.angular_velocity.z),
                            }; 
                            predicted_pose.rotation = quaternion_multiply(quaternion_exp(W), pose.rotation);
                            //  This 'delta pose' calculates the last captured and rendered frame, and the current pose
                            if (resetInitialPose) {
                                //                    Debug::Log("Reset the initial pose", Color::Green);
                                resetInitialPose = false;
                                PoseInitial = glm::toMat4(glm::qua<float>(predicted_pose.rotation.w, predicted_pose.rotation.x, -predicted_pose.rotation.y, -predicted_pose.rotation.z));
                                PoseInitial[3][0] = predicted_pose.translation.x;
                                PoseInitial[3][1] = -predicted_pose.translation.y;
                                PoseInitial[3][2] = predicted_pose.translation.z;
                            } 
                            //  Once delta pose is calculated, call the function pointer callback
                            PoseFinal = glm::toMat4(glm::qua<float>(predicted_pose.rotation.w, predicted_pose.rotation.x, -predicted_pose.rotation.y, -predicted_pose.rotation.z));
                            PoseFinal[3][0] = predicted_pose.translation.x;
                            PoseFinal[3][1] = -predicted_pose.translation.y;
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
                        }
                        now = std::chrono::system_clock::now().time_since_epoch();
                        last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                    }

                    if (auto fs = frame.as<rs2::frameset>()) {// For camera frames
                        hasImage = true;
                        if (receivedPoseFirst) {//Ensure we've received a pose first
                            LockImage = true;
                            rs2::video_frame video_frame = frame.as<rs2::frameset>().get_fisheye_frame(1);
                            const int w = video_frame.get_width();
                            const int h = video_frame.get_height();
                            LockImage = false;
                        }

                    }
                    if (hasPose && hasImage) {
                        Debug::Log("Has Both Pose and Image");
                    }
                    });
                shouldRestart = false;
                while (!shouldRestart && !ExitThreadLoop) {
                    if (grabOriginPose) {
                        grabOriginPose = false;
                        rs2_pose object_in_world_pose_frame; 
                        if (tm_sensor.get_static_node("bugorigin", object_in_world_pose_frame.translation, object_in_world_pose_frame.rotation)) {
                            if (callbackObjectPoseReceived != nullptr) {
                                callbackObjectPoseReceived(TrackerID, "origin_of_map", object_in_world_pose_frame.translation.x, object_in_world_pose_frame.translation.y, object_in_world_pose_frame.translation.z, object_in_world_pose_frame.rotation.x, object_in_world_pose_frame.rotation.y, object_in_world_pose_frame.rotation.z, object_in_world_pose_frame.rotation.w);
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
        Debug::Log("Closed Tracker!", Color::Green);
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
        //This asynchronous thread tells the GPU to copy the texture from cpu memory to gpu, allowing it to display within unity
        //This is called automagically by the unity LLAPI
#ifdef __linux__ //OGL 

#else 

        if (!LockImage) {
            if (m_Device != nullptr) {
                if (hasReceivedTexture) {
                }
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
        hasReceivedTexture = true;
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
        Debug::Log("Finished setting transforms", Color::Green);
    }

};