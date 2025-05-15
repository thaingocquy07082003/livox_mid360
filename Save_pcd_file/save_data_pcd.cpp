#include <livox_lidar_api.h>
#include <livox_lidar_def.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <cassert>

class LivoxToPCD {
public:
  LivoxToPCD(const std::string& config_path) : config_path_(config_path) {
    // Khởi tạo cloud_
    cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
    if (!cloud_) {
      std::cerr << "Khởi tạo PCL cloud thất bại!" << std::endl;
      initialized_ = false;
      return;
    }

    // Khởi tạo Livox SDK2
    std::cout << "Khởi tạo SDK với config: " << config_path_ << std::endl;
    if (!LivoxLidarSdkInit(config_path_.c_str())) {
      std::cerr << "Khởi tạo Livox SDK thất bại!" << std::endl;
      initialized_ = false;
      return;
    }
    initialized_ = true;

    // Đăng ký callback
    SetLivoxLidarPointCloudCallBack(PointCloudCallback, this);

    // Lưu thời gian bắt đầu
    start_time_ = std::chrono::steady_clock::now();
  }

  ~LivoxToPCD() {
    if (initialized_) {
      // Lưu cloud_ cuối cùng
      SaveCloudToPCD();
      LivoxLidarSdkUninit();
    }
  }

  static void PointCloudCallback(uint32_t handle, const uint8_t dev_type, 
                                LivoxLidarEthernetPacket* data, void* client_data) {
    if (client_data == nullptr) {
      std::cerr << "client_data là nullptr!" << std::endl;
      return;
    }
    if (data == nullptr) {
      std::cout << "Dữ liệu từ Mid-360 LiDAR là nullptr" << std::endl;
      return;
    }

    auto* node = static_cast<LivoxToPCD*>(client_data);
    node->ProcessPointCloud(handle, data);
  }

  void ProcessPointCloud(uint32_t handle, LivoxLidarEthernetPacket* data) {
    assert(data != nullptr);
    // std::cout << "Handle: " << handle << ", Dot num: " << data->dot_num
    //           << ", Data type: " << (int)data->data_type
    //           << ", Frame count: " << data->frame_cnt
    //           << ", Total points: " << cloud_->size() << std::endl;

    if (data->dot_num == 0 || data->data == nullptr) {
      std::cout << "Gói dữ liệu rỗng hoặc không hợp lệ!" << std::endl;
      return;
    }

    // Tích lũy điểm
    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
      LivoxLidarCartesianHighRawPoint* p_point_data = 
          (LivoxLidarCartesianHighRawPoint*)data->data;
      for (uint32_t i = 0; i < data->dot_num; i++) {
        pcl::PointXYZI pt;
        pt.x = p_point_data[i].x / 1000.0f; // mm to m
        pt.y = p_point_data[i].y / 1000.0f;
        pt.z = p_point_data[i].z / 1000.0f;
        pt.intensity = p_point_data[i].reflectivity;
        cloud_->push_back(pt);
      }
    } else if (data->data_type == kLivoxLidarCartesianCoordinateLowData) {
      LivoxLidarCartesianLowRawPoint* p_point_data = 
          (LivoxLidarCartesianLowRawPoint*)data->data;
      for (uint32_t i = 0; i < data->dot_num; i++) {
        pcl::PointXYZI pt;
        pt.x = p_point_data[i].x / 1000.0f;
        pt.y = p_point_data[i].y / 1000.0f;
        pt.z = p_point_data[i].z / 1000.0f;
        pt.intensity = p_point_data[i].reflectivity;
        cloud_->push_back(pt);
      }
    } else if (data->data_type == kLivoxLidarSphericalCoordinateData) {
      LivoxLidarSpherPoint* p_point_data = (LivoxLidarSpherPoint*)data->data;
      for (uint32_t i = 0; i < data->dot_num; i++) {
        float theta = p_point_data[i].theta / 180.0f * M_PI;
        float phi = p_point_data[i].phi / 180.0f * M_PI;
        float depth = p_point_data[i].depth / 1000.0f;
        pcl::PointXYZI pt;
        pt.x = depth * std::sin(theta) * std::cos(phi);
        pt.y = depth * std::sin(theta) * std::sin(phi);
        pt.z = depth * std::cos(theta);
        pt.intensity = p_point_data[i].reflectivity;
        cloud_->push_back(pt);
      }
    } else {
      std::cout << "Loại dữ liệu không hỗ trợ: " << (int)data->data_type << std::endl;
      return;
    }
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
    if(elapsed >=120) {
      SaveCloudToPCD();
      LivoxLidarSdkUninit();
    }
  }

  void SaveCloudToPCD() {
    if (cloud_->empty()) {
      std::cout << "Cloud rỗng, không lưu PCD!" << std::endl;
      return;
    }

    std::string filename = "data_final.pcd";
    if (pcl::io::savePCDFileBinary(filename, *cloud_) < 0) {
      std::cerr << "Lưu PCD thất bại: " << filename << std::endl;
    } else {
      std::cout << "Đã lưu " << cloud_->size() << " điểm vào " << filename << std::endl;
    }
  }

  void Run() {
    if (!initialized_) {
      std::cerr << "SDK chưa khởi tạo, không thể chạy!" << std::endl;
      return;
    }

    // Chạy trong 2 phút (120 giây)
    
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Lưu point cloud cuối cùng trước khi thoát
    // SaveCloudToPCD();
  }

private:
  std::string config_path_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  uint32_t frame_count_ = 0;
  bool initialized_ = false;
  std::chrono::steady_clock::time_point start_time_;
};

int main() {
  std::string config_path = "/home/thaingocquy/livox_mid360_project/mid360_config.json";
  LivoxToPCD livox(config_path);
  livox.Run();
  return 0;
}