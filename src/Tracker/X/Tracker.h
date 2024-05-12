#include <cstdlib>
#include <thread>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <mutex>
#include <signal.h>
#include <cstring>
#include <glm/common.hpp>
#include <glm/mat4x4.hpp>
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
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
#include <xv-sdk.h>
#include <cmath>

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
        return latestPoseExternal;
    }
    double* GetOffsetTimestampPose(double msOffset) {//in theory, by extrapolating the sensor values we should get the pose at the lateset time stamp, can't be arsed translating this to the shared math library, will use intels for now
        auto now = std::chrono::system_clock::now().time_since_epoch();
        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

        float dt_s = static_cast<float>(max(0.0, (now_ms - lastPoseTimeStamp) / 1000.0) + msOffset);
        return latestPoseExternal;
    }
    double last_ms = 0;
    
    void show_pose_quaternion(const xv::Pose& pose)
    {
        latestPoseExternal[0] = pose.translation()[0];
        latestPoseExternal[1] = pose.translation()[1];
        latestPoseExternal[2] = pose.translation()[2];
        latestPoseExternal[3] = pose.quaternion()[0];
        latestPoseExternal[4] = pose.quaternion()[1];
        latestPoseExternal[5] = pose.quaternion()[2];
        latestPoseExternal[6] = pose.quaternion()[3];
        if (resetInitialPose) {
            //                    Debug::Log("Reset the initial pose", Color::Green);
            resetInitialPose = false;
            PoseInitial = glm::toMat4(glm::qua<float>(latestPoseExternal[6], latestPoseExternal[3], -latestPoseExternal[4], -latestPoseExternal[5]));
            PoseInitial[3][0] = latestPoseExternal[0];
            PoseInitial[3][1] = -latestPoseExternal[1];
            PoseInitial[3][2] = -latestPoseExternal[2];
        }
        PoseFinal = glm::toMat4(glm::qua<float>(latestPoseExternal[6], latestPoseExternal[3], -latestPoseExternal[4], -latestPoseExternal[5]));
        PoseFinal[3][0] = latestPoseExternal[0];
        PoseFinal[3][1] = -latestPoseExternal[1];
        PoseFinal[3][2] = -latestPoseExternal[2];
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


    void lost(float timestamp)
    {
        ostringstream oss;
        oss << "[LOST] Device is lost at timestamp " << timestamp << " sec." << std::endl;
        Debug::Log(oss.str(), Color::Red);
    }
    int main(int argc, char** argv)
    {
          
    }
    void DoFunctionTracking() {
        double tempx, tempy, tempz = 0;
        double tempw = 1;
        while (!ExitThreadLoop) {
            try {
                Debug::Log("Starting Thread", Color::Green);
                poseFilter = OneDollaryDooFilterPose(tfreq, tmincutoff, tbeta, tdcutoff);
                

                poseFilter.UpdateTranslationParams(tfreq, tmincutoff, tbeta, tdcutoff);
                poseFilter.UpdateRotationParams(rfreq, rmincutoff, rbeta, rdcutoff);
                poseFilter.SetFilterEnabled(filterEnabled);
                poseFilterK.SetDefault(qt, rt, qr, rr);
                poseFilterK.SetFilterEnabled(filterEnabled);
                hasReceivedCameraStream = false;

                if (initializeWithPassthrough) {

                }
                if (shouldUploadData) {
                    shouldUploadData = false;
                    try {
                        //Load data
                        if (true) {
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
                std::atomic_bool stop(false); 
                Debug::Log("Setting up callbacks",Color::Green);
                // function and configuration to get the 3D grid
                Debug::Log("STARTING SENSOR");
                std::string json = "";
                auto devices = xv::getDevices(10.0);
                xv::setLogLevel(xv::LogLevel::debug);
                std::ofstream ofs;
                if (devices.empty())
                {
                    Debug::Log("No Devices Found, Returning", Color::Red);
                    return;
                }
                auto device = devices.begin()->second;
                shouldRestart = false;
                Debug::Log("Entering Managment Loop", Color::Green);
                device->slam()->registerCallback([this](const xv::Pose& pose) {
                    show_pose_quaternion(pose);
                    });
                if (device->slam()->start(xv::Slam::Mode::Mixed)) {
                    Debug::Log("Successfully Initialized XVisioSensor", Color::Green);

                }
                else {
                    Debug::Log("Issue Initializing XVisioSensor", Color::Green);
                }
                while (!shouldRestart && !ExitThreadLoop) {                        
                    
                    if (grabOriginPose) {
                        grabOriginPose = false;
                    }
                    //need to get callback saying we have images ready to go!

                    //Do we need to grab the current local map?
                    if (shouldGrabMap) {
                        shouldGrabMap = false;
                        try {
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
                }
                Debug::Log("Exiting Managment Loop, first stopping sensor",Color::Green);
                device->slam()->stop();
                if (ExitThreadLoop) {
                    break;
                }
            }
            catch (const std::exception& e)
            {
                std::stringstream ss;
                std::cerr << e.what() << std::endl;
                mapDataLoaded = false;
                Debug::Log(ss.str(), Color::Red);
            }

        }
        Debug::Log("Closed Tracker!", Color::Green);
    }
    void ResetInitialPose() {
        resetInitialPose = true;
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

/*        if (!LockImage) {
            if (m_Device != nullptr) {
                if (hasReceivedTexture) {
                    ID3D11DeviceContext* ctx = NULL;
                    m_Device->GetImmediateContext(&ctx);
                    if (ctx != nullptr) {
                        if (d3dtex != nullptr) {
                            ctx->UpdateSubresource(d3dtex, 0, 0, fisheye_mat_color_undistort.data, fisheye_mat_color_undistort.step[0], (UINT)fisheye_mat_color_undistort.total());
                        }
                        ctx->Release();
                    }
                }
            }
        }*/
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