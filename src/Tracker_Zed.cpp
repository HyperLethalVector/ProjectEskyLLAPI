
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
#include <sl/Camera.hpp>
#include <d3d11.h>
#include "IUnityGraphicsD3D11.h"
#include "IUnityInterface.h"
#include "IUnityGraphics.h"
using namespace sl;
class TrackerObject {
public:
    typedef void(*FuncCallBack2)(int LocalizationDelegate);
    typedef void(*FuncCallBack3)(unsigned char* binaryData, int Length);
    typedef void(*FuncCallBack4)(string ObjectID, float tx, float ty, float tz, float qx, float qy, float qz, float qw);
    typedef void(*FuncMeshCallback)(int ChunkID, float* vertices, int verticesLength, float* normals, int normalsLength, float* uvs, int uvsLength, int* triangleIndicies, int triangleIndiciesLength);
    typedef void(*FuncTextureInitializedCallback)(int TextureWidth, int TextureHeight,int textureCount,float v_fov);
    typedef void(*FuncMeshCompleteCallback)();
    FuncTextureInitializedCallback textureInitializedCallback = nullptr;
    FuncCallBack2 callbackLocalization = nullptr;
    FuncCallBack3 callbackBinaryMap = nullptr;
    FuncCallBack4 callbackObjectPoseReceived = nullptr;
    FuncMeshCallback meshCallback = nullptr;
    FuncMeshCompleteCallback meshCompleteCallback = nullptr;

    ID3D11Device* m_Device;
    sl::Mat currentImage;
    sl::Mat currentGPUImage;
    float* pose = new float[] {0, 0, 0, 0, 0, 0, 0};
    float* poseFromWorldToMap = new float[] {0, 0, 0, 0, 0, 0, 0};
    bool hasLocalized = false;
    bool shouldStartTracker = false;
    int chunkSizes = 45;
    unsigned char* fileLocation;
    int textureWidth = 0;
    int textureHeight = 0;
    int textureChannels = 0;
    vector<string> posesToUpdate;
    
    bool shouldStartSpatialMapping = false;
    bool shouldStopSpatialMapping = false;
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
        Debug::Log("Should try to save the info now");
    }
    void ProcessPoseCache() {

    }
    void StartSpatialMapping(int chunkVertexSizes) {
        chunkSizes = chunkVertexSizes;
        shouldStartSpatialMapping = true;
    }
    void StopSpatialMapping() {
        shouldStopSpatialMapping = true;
    }
    bool file_exists(const char* fileName)
    {
        std::ifstream infile(fileName);
        return infile.good();
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
    void FreeToGrabChunks() {
        shouldRequestNewChunks = true;
    }
    bool shouldRequestNewChunks = true;
    bool isFirst = true;
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
            ss << "Map size " << mapDataToLoad.size()<< std::endl;
            Debug::Log(ss.str(), Color::Orange);
            shouldLoadMap = true;
    }
    ID3D11Texture2D* d3dtex;
    sl::SPATIAL_MAPPING_STATE mapping_state;
    bool mapping_activated = false;
    bool shouldUploadData = false;
    bool LockImage = false;
    void UpdatecameraTexture() {

        if (m_Device != nullptr) {
            ID3D11DeviceContext* ctx = NULL;
            m_Device->GetImmediateContext(&ctx);
            //UpdateMarker
            LockImage = true;
            if (d3dtex != nullptr) {
                ctx->UpdateSubresource(d3dtex, 0, NULL, currentImage.getPtr<sl::uchar1>(), textureWidth * textureChannels, 0);
            }
            LockImage = false;
            ctx->Release();
        }
    }
    void DoFunctionTracking() {

        //The outer loop (allows for device graceful restarts)
        while (!DoExit3) {
            try {
                Camera zed;
                POSITIONAL_TRACKING_STATE tracking_state = POSITIONAL_TRACKING_STATE::OFF;
                // Set configuration parameters
                InitParameters init_params;
                init_params.camera_resolution = RESOLUTION::VGA; // Use HD720 video mode (default fps: 60)
                init_params.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
                init_params.coordinate_units = UNIT::METER; // Set units in meters
                chrono::high_resolution_clock::time_point ts_last;

                Debug::Log("Attempting to start the tracker up", Color::Green);
                Debug::Log("Tracker started!", Color::Green);
                shouldRestart = false;
                ERROR_CODE err = zed.open(init_params);
                if (err != ERROR_CODE::SUCCESS) {
                    ostringstream oss;
                    oss << "Error opening zed: " << err << ".\n";
                    Debug::Log(oss.str(), Color::Green);
                }
                PositionalTrackingParameters tracking_parameters;
                if (file_exists("temp.raw.area")) {
                    tracking_parameters.area_file_path = "temp.raw.area";
                }
                err = zed.enablePositionalTracking(tracking_parameters);
                if (err != ERROR_CODE::SUCCESS) {
                    ostringstream oss;
                    oss << "Error enabling positional tracking: " << err << ".\n";
                    Debug::Log(oss.str(), Color::Green);
                }
                bool zed_has_imu = (zed.getCameraInformation().camera_model != MODEL::ZED);
                SensorsData sensor_data;
                Pose zed_pose;
                SPATIAL_MAPPING_STATE mapping_state = SPATIAL_MAPPING_STATE::NOT_ENABLED;
                sl::Mesh map;
                std::vector<float> verticesToReturn;
                std::vector<float> normalsToReturn;
                std::vector<float> UVsToReturn;
                std::vector<int> triangleIndicies;
                bool zedRequestSuccess = false;
                while (!shouldRestart) {
                    if (shouldStartSpatialMapping) { 
                        Debug::Log("ZED: Starting Spatial Mapping");
                        shouldStartSpatialMapping = false;
                        SpatialMappingParameters spatial_mapping_parameters;
                        spatial_mapping_parameters.resolution_meter = SpatialMappingParameters::get(SpatialMappingParameters::MAPPING_RESOLUTION::MEDIUM);
                        spatial_mapping_parameters.use_chunk_only = false;
                        spatial_mapping_parameters.range_meter = SpatialMappingParameters::get(SpatialMappingParameters::MAPPING_RANGE::MEDIUM);
                        spatial_mapping_parameters.save_texture = false;
                        spatial_mapping_parameters.map_type = SpatialMappingParameters::SPATIAL_MAP_TYPE::MESH;
                        if (zed.enableSpatialMapping(spatial_mapping_parameters) == ERROR_CODE::SUCCESS) {
                            mapping_activated = true;
                            Debug::Log("ZED: Spatial Mapping Started!", Color::Green);
                        }
                        else {
                            Debug::Log("ZED: Spatial Mapping couldn't be Started", Color::Red);
                        }
                    }
                    if (zed.grab() == ERROR_CODE::SUCCESS) {
                        zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
                        zedRequestSuccess = true;
                        sl::Translation zed_translation = zed_pose.getTranslation();
                        sl::Rotation zed_rotation = zed_pose.getRotation();
                        int ts = zed_pose.timestamp.getNanoseconds();
                        if (zed_has_imu) {
                            // Get IMU data at the time the image was captured
                            zed.getSensorsData(sensor_data, TIME_REFERENCE::CURRENT);

                            //get filtered orientation quaternion
                            sl::Orientation imu_orientation = sensor_data.imu.pose.getOrientation();
                            // get raw acceleration
                            sl::float3 acceleration = sensor_data.imu.linear_acceleration;
                            //should do some sensor fusion here maybe....
                        }
                        pose[0] = zed_translation.x;
                        pose[1] = zed_translation.y;
                        pose[2] = zed_translation.z;
                        pose[3] = zed_rotation.getOrientation().x;
                        pose[4] = zed_rotation.getOrientation().y;
                        pose[5] = zed_rotation.getOrientation().z;
                        pose[6] = zed_rotation.getOrientation().w;
                    }
                    else { zedRequestSuccess = false; }
                    if (mapping_activated) {
                        mapping_state = zed.getSpatialMappingState();
                        auto duration = chrono::duration_cast<chrono::milliseconds>(chrono::high_resolution_clock::now() - ts_last).count();
                        if ((duration > 2000 && shouldRequestNewChunks)) {
                            shouldRequestNewChunks = false;
                            zed.requestSpatialMapAsync();
                            ts_last = chrono::high_resolution_clock::now();
                        }
                        if (zed.getSpatialMapRequestStatusAsync() == ERROR_CODE::SUCCESS) {
                            if (meshCallback != nullptr && meshCompleteCallback != nullptr) {
                                zed.retrieveSpatialMapAsync(map);
                                for (int i = 0; i < map.chunks.size(); i++) {
                                    verticesToReturn.clear();
                                    normalsToReturn.clear();
                                    UVsToReturn.clear();
                                    triangleIndicies.clear();
                                    for (int j = 0; j < map.chunks[i].vertices.size(); j++) {
                                        verticesToReturn.push_back(map.chunks[i].vertices[j].x);
                                        verticesToReturn.push_back(map.chunks[i].vertices[j].y);
                                        verticesToReturn.push_back(map.chunks[i].vertices[j].z);
                                    }
                                    for (int j = 0; j < map.chunks[i].normals.size(); j++) {
                                        normalsToReturn.push_back(map.chunks[i].normals[j].x);
                                        normalsToReturn.push_back(map.chunks[i].normals[j].y);
                                        normalsToReturn.push_back(map.chunks[i].normals[j].z);
                                    }
                                    for (int j = 0; j < map.chunks[i].uv.size(); j++) {
                                        normalsToReturn.push_back(map.chunks[i].uv[j].x);
                                        normalsToReturn.push_back(map.chunks[i].uv[j].y);
                                    }
                                    for (int j = 0; j < map.chunks[i].triangles.size(); j++) {
                                        for (int k = 0; k < map.chunks[i].triangles[j].size(); k++) {
                                            triangleIndicies.push_back(map.chunks[i].triangles[j][k]);
                                        }
                                    }
                                    if (verticesToReturn.size() > 0 && map.chunks[i].has_been_updated) {
                                        meshCallback(i, verticesToReturn.data(), verticesToReturn.size(), normalsToReturn.data(), normalsToReturn.size(), UVsToReturn.data(), UVsToReturn.size(), triangleIndicies.data(), triangleIndicies.size());
                                    }
                                }
                                meshCompleteCallback();
                            }
                        }
                    }
                    if(zedRequestSuccess){
                        if (!LockImage) {
                            if (zed.retrieveImage(currentImage, sl::VIEW::LEFT, sl::MEM::CPU) == sl::ERROR_CODE::SUCCESS) {
                                if (textureWidth == 0) {//we are unitialized 
                                    textureWidth = currentImage.getWidth();
                                    textureHeight = currentImage.getHeight();
                                    textureChannels = currentImage.getChannels();
                                    sl::CameraInformation cameraInfo = zed.getCameraInformation();
                                    if (textureInitializedCallback != nullptr) {
                                        textureInitializedCallback(textureWidth, textureHeight, textureChannels, cameraInfo.calibration_parameters.left_cam.v_fov);
                                    }
                                }
                            }
                        }
                    }
                    if (shouldStopSpatialMapping) {
                        mapping_activated = false;
                        shouldStopSpatialMapping = false;
                        Debug::Log("ZED: Disabling Spatial Mapping!", Color::Green);
                        zed.disableSpatialMapping();
                    }
                    if (shouldUploadData) {
                        shouldUploadData = false;
                        try {
                            Debug::Log("Loading Map");
                            //
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
                    posesToUpdate.clear();
                    //Do we need to grab the current local map?
                    if (shouldGrabMap) {
                        shouldGrabMap = false;                        
                        try {
                            Debug::Log("Saving Map");
                            sl::ERROR_CODE ec = zed.saveAreaMap("temp.raw");
                            if (ec == sl::ERROR_CODE::SUCCESS) {                                
                                callbackBinaryMap(NULL, 0);
                            }
                            else {
                                Debug::Log("Problem saving the map?", Color::Red);
                                Debug::Log(sl::errorCode2str(ec), Color::Red);
                            }
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
                map.clear();
                zed.disableSpatialMapping();
                zed.disablePositionalTracking();
                zed.close();
                Debug::Log("Stopped Tracker, will attempt to restart again");
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
