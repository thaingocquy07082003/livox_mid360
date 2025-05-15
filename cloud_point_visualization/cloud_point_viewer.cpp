#include <livox_lidar_def.h>
#include <livox_lidar_api.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include <chrono>
#include <iostream>

// Mutex để đồng bộ dữ liệu point cloud
std::mutex cloud_mutex;
pcl::PointCloud<pcl::PointXYZI>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZI>);
bool new_cloud_available = false;

// Chuyển đổi Livox point cloud sang PCL
pcl::PointCloud<pcl::PointXYZI>::Ptr ConvertLivoxToPCL(LivoxLidarEthernetPacket* data) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud;
    pcl_cloud->reserve(data->dot_num);
    LivoxLidarCartesianHighRawPoint* p_point_data = 
          (LivoxLidarCartesianHighRawPoint*)data->data;
    for (uint32_t i = 0; i < data->dot_num; ++i) {
        pcl::PointXYZI point;
        point.x = p_point_data[i].x / 1000.0f;
        point.y = p_point_data[i].y / 1000.0f;
        point.z = p_point_data[i].z / 1000.0f;
        point.intensity = p_point_data[i].reflectivity;
        pcl_cloud->push_back(point);
    }
    return pcl_cloud;
}

// Hàm hiển thị point cloud trong luồng riêng
void VisualizePointCloud() {
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Livox Mid-360 Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2); // Tăng kích thước điểm cho Mid-360

    while (!viewer->wasStopped()) {
        {
            std::lock_guard<std::mutex> lock(cloud_mutex);
            if (new_cloud_available && latest_cloud->size() > 0) {
                viewer->removeAllPointClouds();
                viewer->addPointCloud<pcl::PointXYZI>(latest_cloud, "livox_cloud");
                new_cloud_available = false;
            }
        }
        viewer->spinOnce(10); // Cập nhật 100 FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Callback cho point cloud
void OnLidarPointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* point_cloud, void* user_data) {
    if (point_cloud == nullptr || point_cloud->dot_num == 0) return;

    auto pcl_cloud = ConvertLivoxToPCL(point_cloud);
    {
        std::lock_guard<std::mutex> lock(cloud_mutex);
        latest_cloud = pcl_cloud; // Cập nhật đám mây điểm mới nhất
        new_cloud_available = true;
    }
    std::cout << "Received " << point_cloud->dot_num << " points" << std::endl;
}

// Callback cho IMU
void OnLidarImuDataCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* imu_data, void* user_data) {
    // if (imu_data == nullptr) return;
    // LivoxLidarImuRawPoint* imu_data_raw = (LivoxLidarImuRawPoint*)imu_data->data;
    // std::cout << "IMU: Accel (" << imu_data_raw->acc_x << ", " << imu_data_raw->acc_y << ", " << imu_data_raw->acc_z
    //           << "), Gyro (" << imu_data_raw->gyro_x << ", " << imu_data_raw->gyro_y << ", " << imu_data_raw->gyro_z
    //           << "), Timestamp: " << imu_data_raw->timestamp << std::endl;
    printf("Imu data callback : data_num:%u, data_type:%u, length:%u, frame_counter:%u.\n",
        imu_data->dot_num, imu_data->data_type, imu_data->length, imu_data->frame_cnt);
}

int main() {
    const std::string path = "/home/thaingocquy/livox_mid360_project/mid360_config.json";
    // Khởi tạo Livox SDK2
    if (!LivoxLidarSdkInit(path.c_str())) {
        std::cerr << "Livox SDK2 initialization failed!" << std::endl;
        return -1;
    }

    // Đăng ký callback
    SetLivoxLidarPointCloudCallBack(OnLidarPointCloudCallback, nullptr);
    SetLivoxLidarImuDataCallback(OnLidarImuDataCallback, nullptr);

    // Tạo luồng hiển thị point cloud
    std::thread viz_thread(VisualizePointCloud);

    std::cout << "Waiting for data from Mid-360..." << std::endl;

    // Vòng lặp chính
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Giảm tải CPU
    }

    // Hủy khởi tạo SDK
    LivoxLidarSdkUninit();
    viz_thread.join();
    return 0;
}