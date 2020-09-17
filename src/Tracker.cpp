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

class TrackerObject {
public:
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;

    float* pose = new float[] {0, 0, 0, 0, 0, 0, 0};
    float* poseFromWorldToMap = new float[] {0, 0, 0, 0, 0, 0, 0};
    rs2_pose object_in_world_pose_frame;
    bool hasLocalized = false;
    unsigned char* fileLocation;
    map<string, rs2_pose> posesToUpdate;
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
    float* GrabObjectPose(string objectID) {
        if (hasLocalized) {
            //                rs2::pose_sensor tm_sensor;
/*            if (cfg.resolve(pipe).get_device().first<rs2::pose_sensor>().get_static_node(objectID, object_in_world_pose_frame.translation, object_in_world_pose_frame.rotation)) {
                poseFromWorldToMap[0] = object_in_world_pose_frame.translation.x;
                poseFromWorldToMap[1] = object_in_world_pose_frame.translation.y;
                poseFromWorldToMap[2] = object_in_world_pose_frame.translation.z;
                poseFromWorldToMap[3] = object_in_world_pose_frame.rotation.x;
                poseFromWorldToMap[4] = object_in_world_pose_frame.rotation.y;
                poseFromWorldToMap[5] = object_in_world_pose_frame.rotation.z;
                poseFromWorldToMap[6] = object_in_world_pose_frame.rotation.w;
                return poseFromWorldToMap;
            }
            else {
                Debug::Log("ERROR: The object could not be found!");
                poseFromWorldToMap[0] = 0;
                poseFromWorldToMap[1] = 0;
                poseFromWorldToMap[2] = 0;
                poseFromWorldToMap[3] = 0;
                poseFromWorldToMap[4] = 0;
                poseFromWorldToMap[5] = 0;
                poseFromWorldToMap[6] = 1;
                return poseFromWorldToMap;
            }*/
            Debug::Log("Was able to call back, but needs to fire events locally");
        }
        else {
            Debug::Log("ERROR: The map hasn't localized yet!", Color::Red);
            return nullptr;
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
    void GrabMap() {
        if (callbackBinaryMap != nullptr) {
            Debug::Log("Attempting to obtain localization map", Color::Red);
            std::lock_guard<std::mutex> lock(mutex);
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
    void ImportMap(unsigned char* mapData, int length) {

            std::vector<unsigned char> mapDataToLoad2(mapData, mapData + length);
            mapDataToLoad.clear();
            copy(mapDataToLoad2.begin(), mapDataToLoad2.end(), back_inserter(mapDataToLoad));
            Debug::Log("Here We Go!", Color::Green);
            Debug::Log(length, Color::Orange);
            std::stringstream ss;
            ss << "RealSense size " << mapDataToLoad.size()<< std::endl;
            Debug::Log(ss.str(), Color::Orange);
            shouldLoadMap = true;
    }

    bool shouldUploadData = false;

    std::mutex mutex;
    void DoFunctionTracking() {
        try {
            rs2::pipeline pipe;
            rs2::config cfg;
            cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
            rs2::pose_sensor tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
            
            auto callback = [&](const rs2::frame& frame)
            {
                if (!shouldLoadMap && !shouldUploadData && !shouldGrabMap && !shouldRestart) {
                    std::lock_guard<std::mutex> lock(mutex);

                    if (rs2::pose_frame fp = frame.as<rs2::pose_frame>()) {
                        rs2_pose pose_data = fp.get_pose_data();
                        auto now = std::chrono::system_clock::now().time_since_epoch();
                        double now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
                        double pose_time_ms = fp.get_timestamp();
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

                }
            };
            shouldRestart = true;
            bool firstTime = true;
            rs2::pipeline_profile myProf;
            while (!DoExit3) {
                if (shouldRestart) {
                    shouldRestart = false;
                    try {
                        tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
                            if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                                if (callbackLocalization != nullptr) {
                                    callbackLocalization(1);
                                }
                                hasLocalized = true;
                                Debug::Log("Should be sending back a notification of relocalization");
                            }
                            Debug::Log(n.get_description());
                            });
                        Debug::Log("Attempting to start the tracker up",Color::Green);
                        if(firstTime){
                            firstTime = false; 
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            myProf = pipe.start(cfg,callback);
                        }
                        else {
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                            myProf = pipe.start(cfg, callback);
                        }
                    }
                    catch (std::exception e) { 
                        ostringstream oss;
                        oss << "Problem attempting to start the tracker: " << e.what() << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        try {
                            pipe.stop();
                            std::this_thread::sleep_for(std::chrono::seconds(1));
                        }
                        catch (...) {
                            ostringstream oss;
                            oss << "Problem attempting to start the tracker: Unknown" << std::endl;
                            Debug::Log(oss.str(), Color::Red);
                            shouldRestart = true;
                        }
                        shouldRestart = true;
                    }
                }
                if (shouldUploadData) {
                    std::lock_guard<std::mutex> lock(mutex);
                    shouldUploadData = false;
                    try {
                        Debug::Log("Loading Map");
                        tm_sensor.import_localization_map(mapDataToLoad);
                        ostringstream oss;
                        oss << "Map loaded from input data succesfully!" << std::endl;
                        Debug::Log(oss.str(), Color::Green);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        shouldRestart = true;
                    }
                    catch (std::exception e) {
                        ostringstream oss;
                        oss << "Couldn't Load Map: " << e.what() << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        shouldRestart = true;
                    }
                }
                if (shouldGrabMap) {
                    shouldGrabMap = false;
                    try {
                        rs2::calibration_table res = tm_sensor.export_localization_map();
                        callbackBinaryMap(res.data(), res.size());
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        shouldRestart = true;
                    }
                    catch (const std::exception& ex) {
                        // ...
                        ostringstream oss;
                        oss << "Map save failed" << ex.what() << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        shouldRestart = true;
                    }
                    catch (const std::string& ex) {
                        // ...
                        ostringstream oss;
                        oss << "Map save failed" << ex << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        shouldRestart = true;
                    }
                    catch (...) {
                        ostringstream oss;
                        oss << "Map save failed: Unknown" << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        shouldRestart = true;
                    }

                }
                if (shouldLoadMap) {
                    std::lock_guard<std::mutex> lock(mutex);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    tm_sensor.stop();
                    Debug::Log("Stopped Sensor");
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    Debug::Log("Closing Sensor");
                    tm_sensor.close();
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    Debug::Log("Closed Sensor, reinitializing");
                    try {
                        pipe.stop();
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                    }
                    catch (...) {
                        ostringstream oss;
                        oss << "Map save failed: Unknown" << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                        shouldRestart = true;
                    }
                    shouldLoadMap = false;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                    shouldUploadData = true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            Debug::Log("Should we be exiting?");
            tm_sensor.stop();
            tm_sensor.close();
            Debug::Log("Well we are...");
            delete this;

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
};
