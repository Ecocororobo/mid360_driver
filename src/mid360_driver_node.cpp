#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <vector>
#include <mutex>
#include <cstring>

using namespace std::chrono_literals;

struct PointXYZIT {
  float x;
  float y;
  float z;
  float intensity;
  float time;   // seconds (relative)
};

class Driver : public rclcpp::Node {
public:
  Driver() : Node("mid360_driver_node") {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<double>("publish_period_ms", 100.0);

    config_path_ = get_parameter("config_path").as_string();
    publish_period_ms_ = get_parameter("publish_period_ms").as_double();

    if (config_path_.empty()) {
      RCLCPP_FATAL(get_logger(), "config_path is empty");
      rclcpp::shutdown();
      return;
    }

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/livox/imu", 200);

    if (!LivoxLidarSdkInit(config_path_.c_str())) {
      RCLCPP_FATAL(get_logger(), "Livox SDK init failed");
      rclcpp::shutdown();
      return;
    }

    SetLivoxLidarPointCloudCallBack(PointCloudCbStatic, this);
    SetLivoxLidarImuDataCallback(ImuCbStatic, this);
    SetLivoxLidarInfoChangeCallback(InfoCbStatic, this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds((int)publish_period_ms_),
      std::bind(&Driver::PublishPointCloud, this));

    RCLCPP_INFO(get_logger(), "MID-360 driver started");
  }

  ~Driver() override {
    LivoxLidarSdkUninit();
  }

private:
  /* ---------- Utility ---------- */
  static inline uint64_t GetTimestampNs(const LivoxLidarEthernetPacket* data) {
    uint64_t ts = 0;
    memcpy(&ts, data->timestamp, sizeof(uint64_t));
    return ts;
  }

  /* ---------- PointCloud Callback ---------- */
  static void PointCloudCbStatic(uint32_t, uint8_t,
                                LivoxLidarEthernetPacket* data,
                                void* client_data) {
    reinterpret_cast<Driver*>(client_data)->PointCloudCb(data);
  }

  void PointCloudCb(LivoxLidarEthernetPacket* data) {
    if (!data || data->data_type != kLivoxLidarCartesianCoordinateHighData)
      return;

    uint64_t packet_time_ns = GetTimestampNs(data);

    std::lock_guard<std::mutex> lock(mutex_);

    if (!has_base_time_) {
      base_time_ns_ = packet_time_ns;
      has_base_time_ = true;
    }

    auto* pts =
      reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(data->data);

    cloud_buffer_.reserve(cloud_buffer_.size() + data->dot_num);

    for (uint32_t i = 0; i < data->dot_num; ++i) {
      PointXYZIT p;
      p.x = pts[i].x * 0.001f;
      p.y = pts[i].y * 0.001f;
      p.z = pts[i].z * 0.001f;
      p.intensity = static_cast<float>(pts[i].reflectivity);

      // relative time (sec)
      p.time = static_cast<float>(
        (packet_time_ns - base_time_ns_) * 1e-9);

      cloud_buffer_.push_back(p);
    }
  }

  /* ---------- Publish ---------- */
  void PublishPointCloud() {
    std::vector<PointXYZIT> cloud;
    uint64_t stamp_ns = 0;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (cloud_buffer_.empty()) return;

      cloud.swap(cloud_buffer_);
      stamp_ns = base_time_ns_;
      has_base_time_ = false;
    }

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = rclcpp::Time(stamp_ns, RCL_ROS_TIME);
    msg.header.frame_id = "livox_frame";
    msg.height = 1;
    msg.width = cloud.size();
    msg.is_dense = false;
    msg.is_bigendian = false;

    msg.fields.resize(5);

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
    msg.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[3].count = 1;

    msg.fields[4].name = "time";
    msg.fields[4].offset = 16;
    msg.fields[4].datatype = sensor_msgs::msg::PointField::FLOAT32;
    msg.fields[4].count = 1;

    msg.point_step = sizeof(PointXYZIT);
    msg.row_step = msg.point_step * msg.width;
    msg.data.resize(msg.row_step);

    memcpy(msg.data.data(), cloud.data(), msg.row_step);

    cloud_pub_->publish(msg);
  }

  /* ---------- IMU ---------- */
  static void ImuCbStatic(uint32_t, uint8_t,
                          LivoxLidarEthernetPacket* data,
                          void* client_data) {
    reinterpret_cast<Driver*>(client_data)->ImuCb(data);
  }

  void ImuCb(LivoxLidarEthernetPacket* data) {
    if (!data) return;

    auto* imu =
      reinterpret_cast<LivoxLidarImuRawPoint*>(data->data);

    uint64_t ts_ns = GetTimestampNs(data);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = rclcpp::Time(ts_ns, RCL_ROS_TIME);
    msg.header.frame_id = "livox_frame";

    msg.linear_acceleration.x = imu->acc_x;
    msg.linear_acceleration.y = imu->acc_y;
    msg.linear_acceleration.z = imu->acc_z;

    msg.angular_velocity.x = imu->gyro_x;
    msg.angular_velocity.y = imu->gyro_y;
    msg.angular_velocity.z = imu->gyro_z;

    imu_pub_->publish(msg);
  }

  /* ---------- Info ---------- */
  static void InfoCbStatic(uint32_t handle,
                           const LivoxLidarInfo* info,
                           void* client_data) {
    if (!info) return;
    auto* self = reinterpret_cast<Driver*>(client_data);
    RCLCPP_INFO(self->get_logger(),
      "Lidar connected SN=%s", info->sn);
    SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, nullptr, nullptr);
  }

private:
  std::string config_path_;
  double publish_period_ms_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  std::vector<PointXYZIT> cloud_buffer_;

  uint64_t base_time_ns_{0};
  bool has_base_time_{false};
};

/* ---------- main ---------- */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Driver>());
  rclcpp::shutdown();
  return 0;
}
