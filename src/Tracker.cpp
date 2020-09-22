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
#include "common_header.h"

class TrackerObject {
public:
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    typedef void(*FuncCallBack4)(string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;
    FuncCallBack4 callbackObjectPoseReceived = nullptr;

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
            mapDataToLoad.clear();
            mapDataToLoad.assign(mapData, mapData + length);
            Debug::Log("Here We Go!", Color::Green);
            Debug::Log(length, Color::Orange);
            std::stringstream ss;
            ss << "RealSense size " << mapDataToLoad.size()<< std::endl;
            Debug::Log(ss.str(), Color::Orange);
            shouldLoadMap = true;
    }

    bool shouldUploadData = false;
    void DoFunctionTracking() {

        //The outer loop (allows for device graceful restarts)
        while (!DoExit3) {
            try {
                rs2::pipeline pipe;
                rs2::config cfg;
                rs2::pipeline_profile myProf;
                cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
                rs2::pose_sensor tm_sensor = cfg.resolve(pipe).get_device().first<rs2::pose_sensor>();
                //First, should we upload the map?
                if (shouldUploadData) {
                    shouldUploadData = false;
                    try {
                        Debug::Log("Loading Map");

                        tm_sensor.import_localization_map(mapDataToLoad);
                        ostringstream oss;
                        oss << "Map loaded from input data succesfully!" << std::endl;
                        Debug::Log(oss.str(), Color::Green);
                    }
                    catch (std::exception e) {
                        ostringstream oss;
                        oss << "Couldn't Load Map: " << e.what() << std::endl;
                        Debug::Log(oss.str(), Color::Red);
                    }
                }
                //Then let's initialize the sensor and begin tracking
                tm_sensor.set_notifications_callback([&](const rs2::notification& n) {
                    if (n.get_category() == RS2_NOTIFICATION_CATEGORY_POSE_RELOCALIZATION) {
                        hasLocalized = true;
                        if (callbackLocalization != nullptr) {
                            callbackLocalization(1);
                        }
                        else {
                            Debug::Log("The callback was null?");
                        }
                    }
                    });

                Debug::Log("Attempting to start the tracker up", Color::Green);
                myProf = pipe.start(cfg);
                Debug::Log("Tracker started!", Color::Green);
                shouldRestart = false;
                while (!shouldRestart) {
                    rs2::frameset frameset = pipe.wait_for_frames();
                    rs2::pose_frame pose_frame = frameset.get_pose_frame();
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

                    if (posesToUpdate.size() > 0) {
                            posesToUpdate.clear();
                            rs2_pose p;
                            p.translation.x = 0;
                            p.translation.y = 0;
                            p.translation.z = 0;
                            p.rotation.x = 0;
                            p.rotation.y = 0;
                            p.rotation.z = 0;
                            p.rotation.w = 1;
                            if (tm_sensor.set_static_node("origin_of_map", p.translation,p.rotation)) {
                                ostringstream oss;
                                oss << "Exported pose for: " << "origin" << std::endl;
                                Debug::Log(oss.str(), Color::Red);
                            }
                            else {
                                ostringstream oss;
                                oss << "Unable to export pose for: " << "origin" << std::endl;
                                Debug::Log(oss.str(), Color::Red);
                            }

                    }
                    posesToUpdate.clear();
                    //Do we need to store the frames?
                        if(grabOriginPose) {
                            grabOriginPose = false;

                            if (tm_sensor.get_static_node("origin_of_map", object_in_world_pose_frame.translation, object_in_world_pose_frame.rotation)) {
                                if (callbackObjectPoseReceived != nullptr) {
                                    callbackObjectPoseReceived("origin", object_in_world_pose_frame.translation.x, object_in_world_pose_frame.translation.y, object_in_world_pose_frame.translation.z, object_in_world_pose_frame.rotation.x, object_in_world_pose_frame.rotation.y, object_in_world_pose_frame.rotation.z, object_in_world_pose_frame.rotation.w);
                                }
                                else {
                                    Debug::Log("Wtf? There's no callback?", Color::Red);
                                }
                            }
                            else {
                                Debug::Log("ERROR: The object could not be found!", Color::Red);
                            }

                    }
                    //Do we need to grab the current local map?
                    if (shouldGrabMap) {
                        shouldGrabMap = false;                        
                        try {
                            rs2::calibration_table res = tm_sensor.export_localization_map();
                            //raw_file_from_bytes("map.raw", res);
                            callbackBinaryMap(res.data(), res.size());
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
                Debug::Log("Stopped Tracker, will attempt to restart again");
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
        Debug::Log("Should we be exiting? Because I'm about to!");        
    }
};
