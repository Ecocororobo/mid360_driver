#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "livox_lidar_def.h"
#include "livox_lidar_api.h"
#include <vector>
#include <mutex>

using namespace std::chrono_literals;

class Driver : public rclcpp::Node {
public:
  Driver()
  : Node("mid360_driver_node")
  {
    // パラメータ宣言
    this->declare_parameter<std::string>("config_path", "");

    // パラメータ取得
    std::string config_path = this->get_parameter("config_path").as_string();
    if (config_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Parameter 'config_path' is empty");
      rclcpp::shutdown();
      return;
    }

    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 10);

    if (!LivoxLidarSdkInit(config_path.c_str())) {
      RCLCPP_ERROR(this->get_logger(), "Livox SDK Init Failed (config: %s)", config_path.c_str());
      rclcpp::shutdown();
      return;
    }

    SetLivoxLidarPointCloudCallBack(PointCloudCallbackStatic, this);
    SetLivoxLidarImuDataCallback(ImuDataCallbackStatic, this);
    SetLivoxLidarInfoChangeCallback(LidarInfoChangeCallbackStatic, this);

    // 100msごとにバッファから点群をpublish
    timer_ = this->create_wall_timer(
      100ms, std::bind(&Driver::PublishPointCloud, this));
  }

  ~Driver() override {
    LivoxLidarSdkUninit();
    RCLCPP_INFO(this->get_logger(), "Livox SDK Uninit Done");
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex buffer_mutex_;
  std::vector<float> cloud_buffer_; // x,y,z,float(intensity)

  // --- PointCloud Callback ---
  static void PointCloudCallbackStatic(uint32_t handle, uint8_t dev_type,
                                       LivoxLidarEthernetPacket* data, void* client_data) {
    if (!client_data) return;
    Driver* self = reinterpret_cast<Driver*>(client_data);
    self->PointCloudCallback(handle, dev_type, data);
  }

  void PointCloudCallback(uint32_t, uint8_t, LivoxLidarEthernetPacket* data) {
    if (!data) return;

    if (data->data_type == kLivoxLidarCartesianCoordinateHighData) {
      LivoxLidarCartesianHighRawPoint* points =
        reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);

      std::lock_guard<std::mutex> lock(buffer_mutex_);
      cloud_buffer_.reserve(cloud_buffer_.size() + data->dot_num * 4);
      for (uint32_t i = 0; i < data->dot_num; i++) {
        cloud_buffer_.push_back(points[i].x / 1000.0f);
        cloud_buffer_.push_back(points[i].y / 1000.0f);
        cloud_buffer_.push_back(points[i].z / 1000.0f);
        cloud_buffer_.push_back(static_cast<float>(points[i].reflectivity));
      }
    }
  }

  // --- TimerでPublish ---
  void PublishPointCloud() {
    std::vector<float> tmp;
    {
      std::lock_guard<std::mutex> lock(buffer_mutex_);
      if (cloud_buffer_.empty()) return;
      tmp.swap(cloud_buffer_);
    }

    size_t point_count = tmp.size() / 4;
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "livox_frame";
    msg.height = 1;
    msg.width = point_count;
    msg.is_dense = false;
    msg.is_bigendian = false;

    // PointField定義 (x,y,z:float32, intensity:uint16)
    msg.fields.resize(4);
    msg.fields[0].name = "x";
    msg.fields[0].offset = 0;
    msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[0].count = 1;

    msg.fields[1].name = "y";
    msg.fields[1].offset = 4;
    msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[1].count = 1;

    msg.fields[2].name = "z";
    msg.fields[2].offset = 8;
    msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[2].count = 1;

    msg.fields[3].name = "intensity";
    msg.fields[3].offset = 12;
    msg.fields[3].datatype = sensor_msgs::msg::PointField::UINT16;
    msg.fields[3].count = 1;

    msg.point_step = 16; // x,y,z(float32) + intensity(uint16 + padding)
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    uint8_t* ptr = msg.data.data();
    for (size_t i = 0; i < point_count; i++) {
      float* xyz = reinterpret_cast<float*>(ptr + i * msg.point_step);
      xyz[0] = tmp[i * 4 + 0];
      xyz[1] = tmp[i * 4 + 1];
      xyz[2] = tmp[i * 4 + 2];
      uint16_t* intensity = reinterpret_cast<uint16_t*>(ptr + i * msg.point_step + 12);
      *intensity = static_cast<uint16_t>(tmp[i * 4 + 3]);
    }

    pointcloud_pub_->publish(msg);
  }

  // --- IMU Callback ---
  static void ImuDataCallbackStatic(uint32_t handle, uint8_t dev_type,
                                    LivoxLidarEthernetPacket* data, void* client_data) {
    if (!client_data) return;
    Driver* self = reinterpret_cast<Driver*>(client_data);
    self->ImuDataCallback(handle, dev_type, data);
  }

  void ImuDataCallback(uint32_t, uint8_t, LivoxLidarEthernetPacket* data) {
    if (!data) return;

    auto* imu = reinterpret_cast<LivoxLidarImuRawPoint*>(data->data);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "livox_frame";

    msg.linear_acceleration.x = imu->acc_x;
    msg.linear_acceleration.y = imu->acc_y;
    msg.linear_acceleration.z = imu->acc_z;

    msg.angular_velocity.x = imu->gyro_x;
    msg.angular_velocity.y = imu->gyro_y;
    msg.angular_velocity.z = imu->gyro_z;

    imu_pub_->publish(msg);
  }

  // --- Lidar Info Change Callback ---
  static void LidarInfoChangeCallbackStatic(const uint32_t handle,
                                            const LivoxLidarInfo* info,
                                            void* client_data) {
    if (!client_data || !info) return;
    Driver* node = reinterpret_cast<Driver*>(client_data);
    RCLCPP_INFO(node->get_logger(), "Lidar detected handle: %u, SN: %s", handle, info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
