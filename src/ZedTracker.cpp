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
#include <float.h>
#include "common_header.h"
#include <sl/Camera.hpp>

using namespace std;
using namespace sl;
class ZedTrackerObject {
public:
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;

    float* pose = new float[] {0, 0, 0, 0, 0, 0, 0};
    float* poseFromWorldToMap = new float[] {0, 0, 0, 0, 0, 0, 0};
    sl::Mat image;
    unsigned char* fileLocation;
    /*
    inline rs2_quaternion quaternion_exp(rs2_vector v)
    {
        float x = v.x / 2, y = v.y / 2, z = v.z / 2, th2, th = sqrtf(th2 = x * x + y * y + z * z);
        float c = cosf(th), s = th2 < sqrtf(120 * FLT_EPSILON) ? 1 - th2 / 6 : sinf(th) / th;
        rs2_quaternion Q = { s * x, s * y, s * z, c };
        return Q;
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
    }*/
    bool DoExit3 = false;
    void raw_file_from_bytes(const std::string& filename, const std::vector<uint8_t> bytes)
    {

    }

    std::vector<unsigned char> myMapData;
    bool mapDataLoaded = false;
    void SetMapBinary(unsigned char* bytes, int length) {

    }
    void StopTracking() {
        DoExit3 = true;
    }
    bool doSetOriginMapInfo = false;
    void SetOrigin() {
        doSetOriginMapInfo = true;
    }
    void DoFunctionTracking() {
        Camera zed;
        InitParameters init_params;
        init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode (default fps: 60)
        init_params.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
        init_params.coordinate_units = UNIT::METER;
        init_params.sensors_required = true;
        ERROR_CODE err = zed.open(init_params);
        if (err != ERROR_CODE::SUCCESS) {
            Debug::Log("Error opening the camera",Color::Red);
            return;
        }
        PositionalTrackingParameters tracking_parameters;
        err = zed.enablePositionalTracking(tracking_parameters);
        bool zed_has_imu = (zed.getCameraInformation().camera_model != MODEL::ZED);
        SensorsData sensor_data;
        if (err != ERROR_CODE::SUCCESS) {
            Debug::Log("Error Enabling Camera", Color::Red);
            return;
        }
        Pose zed_pose;
        while (!DoExit3) {
            if (zed.grab() == ERROR_CODE::SUCCESS) {

                // Get the pose of the left eye of the camera with reference to the world frame
                zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

                // get the translation information
                auto zed_translation = zed_pose.getTranslation();
                pose[0] = zed_translation.tx;
                pose[1] = zed_translation.ty;
                pose[2] = zed_translation.tz;
                // get the orientation information
                auto zed_orientation = zed_pose.getOrientation();
                pose[3] = zed_orientation.ox;
                pose[4] = zed_orientation.oy;
                pose[5] = zed_orientation.oz;
                pose[6] = zed_orientation.ow;
                // get the timestamp
                auto ts = zed_pose.timestamp.getNanoseconds();
                ostringstream oss;

                // Display the translation and timestamp
                oss << "Camera Translation: {" << zed_translation << "}, Orientation: {" << zed_orientation << "}, timestamp: " << zed_pose.timestamp.getNanoseconds() << "ns\n";
                Debug::Log(oss.str(), Color::Green);
                // Display IMU data
                if (zed_has_imu) {
                    // Get IMU data at the time the image was captured
                    zed.getSensorsData(sensor_data, TIME_REFERENCE::CURRENT);
                    //get filtered orientation quaternion
                    auto imu_orientation = sensor_data.imu.pose.getOrientation();
                    // get raw acceleration
                    auto acceleration = sensor_data.imu.linear_acceleration;                   
                }
//                zed.retrieveImage(image, VIEW::LEFT);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        zed.disablePositionalTracking();
        zed.close();
    }
};