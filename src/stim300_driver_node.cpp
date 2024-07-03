#include "rclcpp/rclcpp.hpp"
#include "driver_stim300.h"
#include "serial_unix.h"
#include <string>
#include "sensor_msgs/msg/imu.hpp"
#include "sam_msgs/msg/topics.hpp"
#include "sam_msgs/msg/links.hpp"
constexpr int defaultSampleRate{ 125 };
constexpr double averageAllanVarianceOfGyro{ 0.0001 * 2 * 0.00046 };
constexpr double averageAllanVarianceOfAcc{ 100 * 2 * 0.0052 };

Eigen::Quaterniond estimate_orientation(const Eigen::Vector3d& acc)
{

	double pitch = atan2(-acc[0], sqrt(acc[1]*acc[1] + acc[2]*acc[2]));
	//double roll = atan2(acc[1], sqrt(acc[0]*acc[0] + acc[2]*acc[2]));
	double roll = atan2(acc[1], acc[2]);

	Eigen::Quaterniond quat = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(0., Eigen::Vector3d::UnitZ());
  
  // double yaw = atan2(-acc[1], acc[0]);
  // double pitch = atan2(acc[2], sqrt(acc[0] * acc[0] + acc[1] * acc[1]));
//
  // Eigen::Quaterniond quat = Eigen::AngleAxisd(0., Eigen::Vector3d::UnitX()) *
                            // Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                            // Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return quat;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("stim300_driver_node");
  node->declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
  std::string imu_path = node->get_parameter("device_name").as_string();
  RCLCPP_INFO(node->get_logger(), "STIM300 IMU path: %s", imu_path.c_str());

  std::string device_name;
  double varianceOfGyro{ 0 };
  double varianceOfAcc{ 0 };

  SerialUnix serial_driver(imu_path);
  DriverStim300 driver_stim300(serial_driver);

  double standardDeivationOfGyro = node->declare_parameter<double>("stanard_deviation_of_gyro", averageAllanVarianceOfGyro);
  double standardDeviatinOfAcc = node->declare_parameter<double>("stanard_deviation_of_acc", averageAllanVarianceOfAcc);
  int sampleRate = node->declare_parameter<int>("sample_rate", defaultSampleRate);

  varianceOfGyro = sampleRate * pow(standardDeivationOfGyro, 2);
  varianceOfAcc = sampleRate * pow(standardDeviatinOfAcc, 2);

  sensor_msgs::msg::Imu imu_msg_template{};
  imu_msg_template.orientation_covariance[0] = 0.05;
  imu_msg_template.orientation_covariance[4] = 0.01;
  imu_msg_template.orientation_covariance[8] = 0.05;
  imu_msg_template.angular_velocity_covariance[0] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[4] = varianceOfGyro;
  imu_msg_template.angular_velocity_covariance[8] = varianceOfGyro;
  imu_msg_template.linear_acceleration_covariance[0] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[4] = varianceOfAcc;
  imu_msg_template.linear_acceleration_covariance[8] = varianceOfAcc;
  imu_msg_template.orientation.x = 0;
  imu_msg_template.orientation.y = 0;
  imu_msg_template.orientation.z = 0;
  imu_msg_template.header.frame_id = sam_msgs::msg::Links::IMU_LINK;

  auto imuSensorPublisher = node -> create_publisher<sensor_msgs::msg::Imu>(sam_msgs::msg::Topics::STIM_IMU_TOPIC,1000);

  rclcpp::Rate loop_rate(125);

  RCLCPP_INFO(node->get_logger(), "STIM300 IMU initialized successfully");

  while (rclcpp::ok())
  {
    sensor_msgs::msg::Imu stim300msg = imu_msg_template;

    stim300msg.header.stamp = node->get_clock()->now();

    if (driver_stim300.processPacket())
    {
      if (!driver_stim300.isChecksumGood())
      {
        RCLCPP_WARN(node->get_logger(), "stim300 CRC error ");
        continue;
      }

      if (!driver_stim300.isSensorStatusGood())
      {
        RCLCPP_WARN(node->get_logger(), "STIM300: Internal hardware error");
        continue;
      }
      Eigen::Vector3d linear_acceleration = driver_stim300.getAccData();
      Eigen::Vector3d gyro_velocities = driver_stim300.getGyroData();
      Eigen::Vector3d incl_acceleration = driver_stim300.getInclData();

      Eigen::Quaterniond orientation = estimate_orientation(incl_acceleration);

      stim300msg.linear_acceleration.x = linear_acceleration[0];
      stim300msg.linear_acceleration.y = linear_acceleration[1];
      stim300msg.linear_acceleration.z = linear_acceleration[2];

      stim300msg.angular_velocity.x = gyro_velocities[0];
      stim300msg.angular_velocity.y = gyro_velocities[1];
      stim300msg.angular_velocity.z = gyro_velocities[2];

      stim300msg.orientation.x = orientation.x();
      stim300msg.orientation.y = orientation.y();
      stim300msg.orientation.z = orientation.z();
      stim300msg.orientation.w = orientation.w();

      // stim300msg.linear_acceleration.x = driver_stim300.getAccX() + 0.0023;
      // stim300msg.linear_acceleration.y = driver_stim300.getAccY() + 0.05;
      // stim300msg.linear_acceleration.z = driver_stim300.getAccZ() + 0.027;
      // stim300msg.angular_velocity.x = driver_stim300.getGyroX();
      // stim300msg.angular_velocity.y = driver_stim300.getGyroY();
      // stim300msg.angular_velocity.z = driver_stim300.getGyroZ();
      imuSensorPublisher->publish(stim300msg);
    }

    loop_rate.sleep();

    rclcpp::spin_some(node);
  }
  return 0;
}