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
struct Quaternion {
    double w, x, y, z;
};

struct EulerAngles {
    double z, x, y;
};
class TrackerObject {
public:
    typedef void(*QuaternionCallback)(float* arrayToCopy, float eux, float euy, float euz);
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    typedef void(*FuncCallBack4)(string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
    typedef void(*FuncTextureInitializedCallback)(int TextureWidth, int TextureHeight, int textureCount, float fx, float fy, float cx, float cy, float fovx, float fovy, float focalLength);
    typedef void(*FuncMatrixDeltaConvert)(float* matrixToReturn, float* inverseToReturn, float tx_A, float ty_A, float tz_A, float qx_A, float qy_A, float qz_A, float qw_A, float tx_B, float ty_B, float tz_B, float qx_B, float qy_B, float qz_B, float qw_B);

    typedef void(*FuncDeltaPoseUpdateCallback)(float *poseData,float *poseDataInv, int length);
    rs2_intrinsics intrinsics;
    QuaternionCallback quaternionCallback = nullptr;
    FuncTextureInitializedCallback textureInitializedCallback = nullptr;
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;
    FuncCallBack4 callbackObjectPoseReceived = nullptr;
    FuncMatrixDeltaConvert callbackMatrixConvert = nullptr;
    FuncDeltaPoseUpdateCallback callbackDeltaPoseUpdate = nullptr;
#ifdef __linux__ //OGL

#else
    ID3D11Device* m_Device;
    ID3D11Texture2D* d3dtex;
#endif
    bool resetInitialPose = true;
    bool LockImage = false;
    float* pose = new float[7] {0, 0, 0, 0, 0, 0, 0};
    float* poseInitial = new float[7]{ 0,0,0,0,0,0,0 };


    float* poseFromWorldToMap = new float[7] {0, 0, 0, 0, 0, 0, 0};
    float* deltaPoseArray = new float[16]{ 1,0,0,0,
                                     0,1,0,0,
                                     0,0,1,0,
                                     0,0,0,1 };
    float* deltaPoseInvArray = new float[16]{ 1,0,0,0,
                                     0,1,0,0,
                                     0,0,1,0,
                                     0,0,0,1 };
    float* hardCodedProjection = new float[16]{
                                                1.427334, 0,    0.5, 0.0,
                                                0  , 2.537483f, 0.5, 0.0,
                                                0  , 0   , -1.0, -0.2,
                                                0  , 0   , -1.0, 1.0
                                            };
    float* hardCodedExtrinsic = new float[16]{
                                            1.0,0.0,0.0,0.0,
                                            0.0,1.0,0.0,0.0,
                                            0.0,0.0,1.0,0.0,
                                            0.0,0.0,0.0,1.0
    };

    //get intrinsics as float array
    // the fisheye mat
    cv::Mat fisheye_mat;
    cv::Mat fisheye_mat_color;
    cv::Mat fisheye_mat_color_undistort;
    cv::Mat lm1, lm2;
    cv::Mat affineTransform;
    rs2_pose object_in_world_pose_frame;
    EulerAngles eulerAnglesPrev;
    cv::Point3f transPrev;

    EulerAngles eulerAnglesPost;
    cv::Point3f transPost;
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
#ifdef __linux__
#else

    DWORD COM_BAUD_RATE = CBR_9600;
    SimpleSerial* Serial; 
#endif
    int nStates = 18;            // the number of states
    int nMeasurements = 6;       // the number of measured states
    int nInputs = 0;             // the number of action control
    double dt = 0.005;           // time between measurements (1/FPS)
    cv::KalmanFilter KF;         // instantiate Kalman Filter
    void ResetInitialPose() {
        resetInitialPose = true;
    }
    Quaternion ToQuaternion(double y, double p, double r) // yaw (Z), pitch (Y), roll (X)
    {
        double yaw = y * (PI / 180.0f);
        double pitch = p * (PI / 180.0f);
        double roll = r * (PI / 180.0f);

        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        Quaternion q;
        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;
        return q;
    }
    void ToEulerAngles(Quaternion q, EulerAngles& angles) {
        double sqw = q.w * q.w;
        double sqx = q.x * q.x;
        double sqy = q.y * q.y;
        double sqz = q.z * q.z;
        double unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
        double test = q.x * q.y + q.z * q.w;
        double heading, attitude, bank;
        if (test > 0.499 * unit) { // singularity at north pole
            heading = 2 * atan2(q.x, q.w);
            attitude = PI / 2;
            bank = 0;
            angles.y = heading * (180.0f / PI);
            angles.x = attitude * (180.0f / PI);
            angles.z = bank * (180.0f / PI);
            return;
        }
        if (test < -0.499 * unit) { // singularity at south pole
            heading = -2 * atan2(q.x, q.w);
            attitude = -PI / 2;
            bank = 0;
            angles.y = heading * (180.0f / PI);
            angles.x = attitude * (180.0f / PI);
            angles.z = bank * (180.0f / PI);
            return;
        }
        heading = atan2(2 * q.y * q.w - 2 * q.x * q.z, sqx - sqy - sqz + sqw);
        attitude = asin(2 * test / unit);
        bank = atan2(2 * q.x * q.w - 2 * q.y * q.z, -sqx + sqy - sqz + sqw);
        angles.y = heading * (180.0f / PI);
        angles.x = attitude * (180.0f / PI);
        angles.z = bank * (180.0f / PI);
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
    void SetIdentity(cv::Mat& inputArr) {
        float ret[4][4];
        ret[0][0] = 1.0f;
        ret[0][1] = 0.0f;
        ret[0][2] = 0.0f;
        ret[0][3] = 0.0f;

        ret[1][0] = 0.0f;
        ret[1][1] = 1.0f;
        ret[1][2] = 0.0f;
        ret[1][3] = 0.0f;

        ret[2][0] = 0.0f;
        ret[2][1] = 0.0f;
        ret[2][2] = 1.0f;
        ret[2][3] = 0.0f;
        std::memcpy(inputArr.data, ret, 4 * 3 * sizeof(float));
    }
    cv::Point2f MultiplyPoint(cv::Mat projection, cv::Mat intrinsics, cv::Mat deltaPose, const cv::Point2f point) //multiply RenderPlane point by the deltaTransformationMatrix
    {         
        cv::Mat_<float> UVCoordinateInitial(4, 1);
        UVCoordinateInitial(0, 0) = point.x;//U
        UVCoordinateInitial(1, 0) = point.y; //V
        UVCoordinateInitial(2, 0) = 1.0;//Weight 1
        UVCoordinateInitial(3, 0) = 1.0;//Weight 2

        cv::Mat homography = intrinsics * projection; //compute the homography
        cv::Mat_<float> TwoDUVto3DSpace = homography * UVCoordinateInitial; //Project our UV point into 3D space
        cv::Mat transformed3DPoint = deltaPose * TwoDUVto3DSpace; // transform according to the delta pose
        cv::Mat UVCoordinateFinal = homography.inv() * transformed3DPoint; //transform 3D point back into UV space
        
        //we have a UV with a modified Z, need to normalize
        float newW = (UVCoordinateFinal.at<float>(3, 0));
        float newZ = (UVCoordinateFinal.at<float>(2, 0))/newW;
        float newX = ((UVCoordinateFinal.at<float>(0, 0))/newW);
        float newY = ((UVCoordinateFinal.at<float>(1, 0))/newW);
        ostringstream oss2;      
        newZ = newZ / newZ;//normalize

        oss2 << "PrePoint: " << point.x << "," << point.y << "," << 1.0 << "," << 1.0 << std::endl;
        oss2 << "PostPoint: " << newX << "," << newY << "," << newZ << "," << newW << std::endl;

        Debug::Log(oss2.str());
        return cv::Point2f(newX, newY); //The 'new' Screen space coordinate
    }
    void CopyTransformationMatrix(cv::Mat& inputArr,float tx, float ty, float tz, float rx, float ry, float rz, float rw)
    {
        float xx = rx * rx;
        float yy = ry * ry;
        float zz = rz * rz;
        float xy = rx * ry;
        float wz = rw * rz;
        float wy = rw * ry;
        float xz = rx * rz;
        float yz = ry * rz;
        float wx = rw * rx;

        float ret[4][4];
        ret[0][0] = 1.0f - 2 * (yy + zz);
        ret[1][0] = 2 * (xy - wz);
        ret[2][0] = 2 * (wy + xz);
        ret[3][0] = tx;

        ret[0][1] = 2 * (xy + wz);
        ret[1][1] = 1.0f - 2 * (xx + zz);
        ret[2][1] = 2 * (yz - wx);
        ret[3][1] = ty;

        ret[0][2] = 2 * (xz - wy);
        ret[1][2] = 2 * (yz + wx);
        ret[2][2] = 1.0f - 2 * (xx + yy);
        ret[3][2] = tz;

        ret[0][3] = 0.0f;
        ret[1][3] = 0.0f;
        ret[2][3] = 0.0f;
        ret[3][3] = 1.0f;

        std::memcpy(inputArr.data, ret, 4 * 4 * sizeof(float));
    }
    cv::Mat eulerAnglesToRotationMatrix(float x, float y, float z)
    {
        // Calculate rotation about x axis
        cv::Mat R_x = (cv::Mat_<float>(3, 3) <<
            1, 0, 0,
            0, cos(x), -sin(x),
            0, sin(x), cos(x)
            );

        // Calculate rotation about y axis
        cv::Mat R_y = (cv::Mat_<float>(3, 3) <<
            cos(y), 0, sin(y),
            0, 1, 0,
            -sin(y), 0, cos(y)
            );

        // Calculate rotation about z axis
        cv::Mat R_z = (cv::Mat_<float>(3, 3) <<
            cos(z), -sin(z), 0,
            sin(z), cos(z), 0,
            0, 0, 1);


        // Combined rotation matrix
        cv::Mat R = R_z * R_y * R_x;
        return R;
    }

    void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
        rs2_pose& predicted, EulerAngles& ea)
    {
        // First predict, to update the internal statePre variable
        cv::Mat prediction = KF.predict();
        // The "correct" phase that is going to use the predicted value and our measurement
        cv::Mat estimated = KF.correct(measurement);
        // Estimated translation
        predicted.translation.x = estimated.at<double>(0);
        predicted.translation.y = estimated.at<double>(1);
        predicted.translation.z = estimated.at<double>(2);
        // Estimated euler angles
//        cv::Mat eulers_estimated(3, 1, CV_64F);
        ea.z = estimated.at<double>(9);
        ea.x = estimated.at<double>(10);
        ea.y = estimated.at<double>(11);
    }
    void fillMeasurements(cv::Mat& measurements,
        float transX,float transY, float transZ, float rotEurX,float rotEurY, float rotEurZ)
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
    void initKalmanFilter(cv::KalmanFilter& KF, int nStates, int nMeasurements, int nInputs, double dt)
    {
        KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
        cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
        cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-4));   // set measurement noise
        cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
        // position
        KF.transitionMatrix.at<double>(0, 3) = dt;
        KF.transitionMatrix.at<double>(1, 4) = dt;
        KF.transitionMatrix.at<double>(2, 5) = dt;
        KF.transitionMatrix.at<double>(3, 6) = dt;
        KF.transitionMatrix.at<double>(4, 7) = dt;
        KF.transitionMatrix.at<double>(5, 8) = dt;
        KF.transitionMatrix.at<double>(0, 6) = 0.5 * pow(dt, 2);
        KF.transitionMatrix.at<double>(1, 7) = 0.5 * pow(dt, 2);
        KF.transitionMatrix.at<double>(2, 8) = 0.5 * pow(dt, 2);
        // orientation
        KF.transitionMatrix.at<double>(9, 12) = dt;
        KF.transitionMatrix.at<double>(10, 13) = dt;
        KF.transitionMatrix.at<double>(11, 14) = dt;
        KF.transitionMatrix.at<double>(12, 15) = dt;
        KF.transitionMatrix.at<double>(13, 16) = dt;
        KF.transitionMatrix.at<double>(14, 17) = dt;
        KF.transitionMatrix.at<double>(9, 15) = 0.5 * pow(dt, 2);
        KF.transitionMatrix.at<double>(10, 16) = 0.5 * pow(dt, 2);
        KF.transitionMatrix.at<double>(11, 17) = 0.5 * pow(dt, 2);
        KF.measurementMatrix.at<double>(0, 0) = 1;  // tx
        KF.measurementMatrix.at<double>(1, 1) = 1;  // ty
        KF.measurementMatrix.at<double>(2, 2) = 1;  // tz
        KF.measurementMatrix.at<double>(3, 9) = 1;  // rx
        KF.measurementMatrix.at<double>(4, 10) = 1; // ry
        KF.measurementMatrix.at<double>(5, 11) = 1; // rz
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
#ifdef __linux__ //OGL

#else //DX
        d3dtex = (ID3D11Texture2D*)textureHandle;
#endif
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
    bool shouldUploadData = false;
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
        while (!DoExit3) {
            try {
                cv::Point2f srcTri[3];
                srcTri[0] = cv::Point2f(0.f, 0.f);
                srcTri[1] = cv::Point2f(1.0f, 0.f);
                srcTri[2] = cv::Point2f(0.0f, 1.0f);

                cv::Point2f dstTri[3];
                dstTri[0] = cv::Point2f(0.f, 0.f);
                dstTri[1] = cv::Point2f(1.0f, 0.f);
                dstTri[2] = cv::Point2f(0.0f, 1.0f);

                cv::Mat projMatVirtual = cv::Mat(4, 4, CV_32F, hardCodedProjection);
                cv::Mat intrinsicsVirtual = cv::Mat(4, 4, CV_32F, hardCodedExtrinsic);
                
                cv::Mat initialPose = cv::Mat(4, 4, CV_32F); SetIdentity(initialPose);
                cv::Mat finalPose = cv::Mat(4, 4, CV_32F); SetIdentity(finalPose);
                cv::Mat deltaPose = cv::Mat(4, 4, CV_32F); SetIdentity(deltaPose);



                cv::Mat aff = cv::Mat(3, 4, CV_32F);
                float AngleToDeg = (3.14f / 180.0f);
                float* f = new float[4] {0.0f, 0.0f, 0.0f, 0.0f};
                initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
                cv::Mat measurements(nMeasurements, 1, CV_64FC1); measurements.setTo(cv::Scalar(0));
                hasReceivedCameraStream = false;
                rs2::pipeline pipe;
                rs2::config cfg;
                rs2::pipeline_profile myProf;
                Quaternion qq;
                EulerAngles eu;
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
                    if (auto fs = frame.as<rs2::pose_frame>()) {// For pose frames
                        rs2::pose_frame pose_frame = frame.as<rs2::pose_frame>();
                        rs2_pose pose_data = pose_frame.get_pose_data();
                        auto now = std::chrono::system_clock::now().time_since_epoch();
                        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                        double pose_time_ms = pose_frame.get_timestamp();
                        float dt_s = static_cast<float>(max(0.0, (now_ms - pose_time_ms) / 1000.0));
                        rs2_pose predicted_pose = predict_pose(pose_data, dt_s);
                        rs2_quaternion qt = predicted_pose.rotation;
                        qq.x = predicted_pose.rotation.x;
                        qq.y = predicted_pose.rotation.y;
                        qq.z = predicted_pose.rotation.z;
                        qq.w = predicted_pose.rotation.w;
                        EulerAngles EulerPre = eu;
                        EulerPre.x = eu.x;
                        EulerPre.y = eu.y;
                        EulerPre.z = eu.z;
                        ToEulerAngles(qq, eu);
                        fillMeasurements(measurements, predicted_pose.translation.x, predicted_pose.translation.y, -predicted_pose.translation.z, eu.x, -eu.y, -eu.z); //add the measurement to the filter
                        updateKalmanFilter(KF, measurements, predicted_pose, eu); // update the predicted value
                        pose[0] = predicted_pose.translation.x;
                        pose[1] = predicted_pose.translation.y;
                        pose[2] = predicted_pose.translation.z;
                        if (std::abs(EulerPre.x - eu.x) > 90 || std::abs(EulerPre.y - eu.y) > 90 || std::abs(EulerPre.z - eu.z) > 90) {//we are jumping rotation poses, reset the kalman filter and use the raw measurements for this frame
                            Debug::Log("Jumping Pose");
                            initKalmanFilter(KF, nStates, nMeasurements, nInputs, dt);
                            measurements.setTo(cv::Scalar(0));
                            ToEulerAngles(qq, eu);
                            fillMeasurements(measurements, predicted_pose.translation.x, predicted_pose.translation.y, -predicted_pose.translation.z, eu.x, -eu.y, -eu.z); //add the measurement to the filter
                            updateKalmanFilter(KF, measurements, predicted_pose, eu); // update the predicted value
                        } 
                        if (quaternionCallback != nullptr) {
                            quaternionCallback(f, eu.x, eu.y, eu.z);
                        }
                        pose[3] = f[0];
                        pose[4] = f[1];
                        pose[5] = f[2]; 
                        pose[6] = f[3];
                        if (resetInitialPose) { //here if we receive the flag (a frame is rendered), we calculate the new pose as the 'initial'
                            resetInitialPose = false;
                            for (int i = 0; i < 7; i++) {
                                poseInitial[i] = pose[i];
                            }
                        }
                        if (callbackMatrixConvert != nullptr) {
                            callbackMatrixConvert(deltaPoseArray,deltaPoseInvArray,
                                poseInitial[0], poseInitial[1], poseInitial[2], poseInitial[3], poseInitial[4], poseInitial[5], poseInitial[6],
                                pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6]
                            );
                        }
                        if (callbackDeltaPoseUpdate != nullptr) {
                            callbackDeltaPoseUpdate(deltaPoseArray,deltaPoseInvArray, 15);
                        } 
                        
                    }  
                    if (auto fs = frame.as<rs2::frameset>()) {// For camera frames
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
//                    cv::warpAffine(img, postWarp, aff, postWarp.size());
  //                  cv::imshow("Pre warped", img);
    //                cv::imshow("warped output ", postWarp); 
                }
                if (usesIntegrator) {
#ifdef __linux__
                    SerialPort serialPort(com_port, BaudRate::B_9600);
                    serialPort.SetTimeout(-1); // Block when reading until any data is received
                    serialPort.Open();
                    serialPort.Write("r\r\n");
                    serialPort.Close();
#else
                    Debug::Log("Attempting to restart after finishing", Color::Green);
                    Serial = new SimpleSerial(com_port, COM_BAUD_RATE);
                    if (Serial->WriteSerialPort("r\r\n")) {
                        Debug::Log("Restarted the t265/1", Color::Green);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                    Serial->CloseSerialPort();
#endif
                }
                pipe.stop(); 
               
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
#ifdef __linux__

#else

        if (usesIntegrator) {
            Serial->CloseSerialPort();
        }
#endif
        Debug::Log("Closed Tracker!",Color::Green);   
    }
};
