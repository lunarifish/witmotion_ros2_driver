/**
 * @file  main.cc
 * @brief ROS2 driver implementing the wit-motion WIT private protocol

 * @note  this driver is written and tested on a HWT606 sensor:
 *        https://wit-motion.yuque.com/wumwnr/docs/bgnf89
 * @note  so no guarantee that it will cope well with other sensors, use with caution.
 *
 * @todo  magnetometer data is not being published
 * @todo  and maybe more parameters to configure the sensor settings
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <cstdint>
#include <functional>
#include <thread>
#include <memory>

#include "serial.h"
#include "wit_c_sdk.h"
#include "REG.h"

using namespace std::chrono_literals;

/**
 * @brief convert std::function to raw function type
 * @note  call this only once, or the static variable gets overwritten
 */
template <typename F>
static auto StdFunctionToFuncPtr(F&& f) -> void (*)(uint32_t, uint32_t) {
  static auto f_static = std::move(f);
  return [](uint32_t uiReg, uint32_t uiRegNum) { f_static(uiReg, uiRegNum); };
}

class WitMotionNode : public rclcpp::Node {
 public:
  WitMotionNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("witmotion_node", options) {
    declare_parameter<std::string>("serial_port", "/dev/ttyCH341USB0");
    declare_parameter<std::string>("tf_parent_frame", "base_link");
    declare_parameter<bool>("broadcast_tf", true);
    get_parameter("serial_port", ros_parameters_.serial_port);
    get_parameter("tf_parent_frame", ros_parameters_.tf_parent_frame);
    get_parameter("broadcast_tf", ros_parameters_.broadcast_tf);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    transform_stamped_.header.frame_id = ros_parameters_.tf_parent_frame;
    transform_stamped_.child_frame_id = "imu";
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", 10);

    if ((serial_fd_ = serial_open(ros_parameters_.serial_port.c_str(), 9600) < 0)) {
      RCLCPP_ERROR(get_logger(), "open %s fail", ros_parameters_.serial_port.c_str());
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(get_logger(), "open %s success", ros_parameters_.serial_port.c_str());
    }

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitRegisterCallBack(
        StdFunctionToFuncPtr([&](uint32_t uiReg, uint32_t uiRegNum) { SensorDataUpdateCallback(uiReg, uiRegNum); }));
    ScanForSensor();

    main_loop_thread_ = std::thread(&WitMotionNode::MainLoop, this);
    RCLCPP_INFO(get_logger(), "publishing imu data on imu topic");
    if (ros_parameters_.broadcast_tf) {
      RCLCPP_INFO(get_logger(), "broadcasting transform: %s -> imu", ros_parameters_.tf_parent_frame.c_str());
    }
  }

  ~WitMotionNode() override { serial_close(serial_fd_); }

 private:
  void MainLoop() {
    unsigned char single_byte_buff;
    while (rclcpp::ok()) {
      while (serial_read(serial_fd_, &single_byte_buff, 1)) {
        WitSerialDataIn(single_byte_buff);
      }
    }
  }

  void SensorDataUpdateCallback(uint32_t uiReg, uint32_t uiRegNum) {
    for (int i = 0; i < uiRegNum; i++) {
      switch (uiReg) {
        case AZ:
          data_updated_.acc = true;
          break;
        case GZ:
          data_updated_.gyro = true;
          break;
        case q3:
          data_updated_.quat = true;
          break;
        default:
          break;
      }
      uiReg++;
    }
    ++valid_transmissions_;
    if (data_updated_.acc && data_updated_.gyro && data_updated_.quat) {
      imu_msg_.header.stamp = now();
      imu_msg_.linear_acceleration.x = sReg[AX] / 32768.0f * 16.0f;
      imu_msg_.linear_acceleration.y = sReg[AY] / 32768.0f * 16.0f;
      imu_msg_.linear_acceleration.z = sReg[AZ] / 32768.0f * 16.0f;
      imu_msg_.angular_velocity.x = sReg[GX] / 32768.0f * 2000.0f;
      imu_msg_.angular_velocity.y = sReg[GY] / 32768.0f * 2000.0f;
      imu_msg_.angular_velocity.z = sReg[GZ] / 32768.0f * 2000.0f;
      imu_msg_.orientation.x = sReg[q1] / 32768.0f;
      imu_msg_.orientation.y = sReg[q2] / 32768.0f;
      imu_msg_.orientation.z = sReg[q3] / 32768.0f;
      imu_msg_.orientation.w = sReg[q0] / 32768.0f;

      // broadcast tf
      if (ros_parameters_.broadcast_tf) {
        transform_stamped_.transform.rotation.x = imu_msg_.orientation.x;
        transform_stamped_.transform.rotation.y = imu_msg_.orientation.y;
        transform_stamped_.transform.rotation.z = imu_msg_.orientation.z;
        transform_stamped_.transform.rotation.w = imu_msg_.orientation.w;
        transform_stamped_.header.stamp = now();
        tf_broadcaster_->sendTransform(transform_stamped_);
      }

      // publish imu
      imu_pub_->publish(imu_msg_);

      // reset flags
      data_updated_.acc = false;
      data_updated_.gyro = false;
      data_updated_.quat = false;
    }
  }

  void ScanForSensor() {
    constexpr int c_uiBaud[] = {2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
    int i, iRetry;
    unsigned char cBuff;

    for (i = 1; i < 10; i++) {
      serial_close(serial_fd_);
      serial_fd_ = serial_open(this->ros_parameters_.serial_port.c_str(), c_uiBaud[i]);
      RCLCPP_INFO(get_logger(), "looking for sensor at %d baud", c_uiBaud[i]);
      iRetry = 2;
      do {
        valid_transmissions_ = 0;
        WitReadReg(AX, 3);
        std::this_thread::sleep_for(10ms);
        while (serial_read(serial_fd_, &cBuff, 1)) {
          WitSerialDataIn(cBuff);
        }
        if (valid_transmissions_ != 0) {
          RCLCPP_INFO(get_logger(), "find sensor @%d baud", c_uiBaud[i]);
          return;
        }
        iRetry--;
      } while (iRetry);
    }

    RCLCPP_ERROR(get_logger(), "can't find sensor");
    rclcpp::shutdown();
  }

 private:
  struct {
    std::string serial_port{};
    std::string tf_parent_frame{};
    bool broadcast_tf{};
  } ros_parameters_{};
  int serial_fd_{};

  struct {
    bool acc{false};
    bool gyro{false};
    bool quat{false};
  } data_updated_{};

  // tf2
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  // imu publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_{nullptr};
  sensor_msgs::msg::Imu imu_msg_;

  std::thread main_loop_thread_;
  uint64_t valid_transmissions_{0};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WitMotionNode>());
  rclcpp::shutdown();
  return 0;
}
