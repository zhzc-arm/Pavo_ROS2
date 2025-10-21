#include "pavo_driver.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <unistd.h>

#define DEG2RAD(x) ((x)*M_PI / 180.)

using namespace std::chrono_literals;
using namespace pavo;

class PavoScanNode : public rclcpp::Node {
public:
  PavoScanNode() : Node("pavo_scan_node") {
    // Declare parameters
    this->declare_parameter<std::string>("frame_id", "laser_frame");
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<double>("angle_max", 135.0);
    this->declare_parameter<double>("angle_min", -135.0);
    this->declare_parameter<double>("range_max", 20.0);
    this->declare_parameter<double>("range_min", 0.10);
    this->declare_parameter<bool>("inverted", false);
    this->declare_parameter<int>("motor_speed", 15);
    this->declare_parameter<int>("merge_coef", 2);
    this->declare_parameter<bool>("enable_motor", true);
    this->declare_parameter<std::string>("lidar_ip", "10.10.10.6");
    this->declare_parameter<int>("lidar_port", 2368);
    this->declare_parameter<std::string>("host_ip", "10.10.10.100");
    this->declare_parameter<int>("host_port", 2368);
    this->declare_parameter<int>("method", 0);
    this->declare_parameter<bool>("switch_active_mode", false);

    // Get parameters
    frame_id = this->get_parameter("frame_id").as_string();
    scan_topic = this->get_parameter("scan_topic").as_string();
    angle_max = this->get_parameter("angle_max").as_double();
    angle_min = this->get_parameter("angle_min").as_double();
    range_max = this->get_parameter("range_max").as_double();
    range_min = this->get_parameter("range_min").as_double();
    inverted = this->get_parameter("inverted").as_bool();
    motor_speed = this->get_parameter("motor_speed").as_int();
    merge_coef = this->get_parameter("merge_coef").as_int();
    enable_motor = this->get_parameter("enable_motor").as_bool();
    lidar_ip = this->get_parameter("lidar_ip").as_string();
    lidar_port = this->get_parameter("lidar_port").as_int();
    host_ip = this->get_parameter("host_ip").as_string();
    host_port = this->get_parameter("host_port").as_int();
    method = this->get_parameter("method").as_int();
    switch_active_mode = this->get_parameter("switch_active_mode").as_bool();

    // Publisher
    scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);

    // Driver init
    drv = switch_active_mode ? new pavo_driver(host_ip, host_port) : new pavo_driver();
    drv->pavo_open(lidar_ip, lidar_port);
    drv->enable_motor(enable_motor);

    if (!enable_motor) {
      kill(getppid(), SIGILL);
      rclcpp::shutdown();
      return;
    }

    if (method >= 0 && method <= 3) {
      drv->enable_tail_filter(method);
      if (method > 0)
        RCLCPP_INFO(this->get_logger(), "Tail filter method: %d", method);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid tail filter method");
      rclcpp::shutdown();
      return;
    }

/*    int motor_speed_get;
    if (drv->get_motor_speed(motor_speed_get)) {
      RCLCPP_INFO(this->get_logger(), "Current motor speed: %d", motor_speed_get);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to get motor speed");
      rclcpp::shutdown();
      return;
    }

    if (motor_speed != motor_speed_get) {
      if (drv->set_motor_speed(motor_speed)) {
        RCLCPP_INFO(this->get_logger(), "Motor speed set to: %d", motor_speed);
        sleep(15);
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to set motor speed");
        rclcpp::shutdown();
        return;
      }
    }
*/
    if (!drv->set_merge_coef(merge_coef)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set merge coefficient");
      rclcpp::shutdown();
      return;
    }
 initialized_ = true;  // 初始化成功
    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / motor_speed),
      std::bind(&PavoScanNode::scan_callback, this)
    );
  }

 bool is_initialized() const {
    return initialized_;
  }

  ~PavoScanNode() {
    delete drv;
  }

private:
  void scan_callback() {
    std::vector<pavo_response_scan_t> scan_vec;
    rclcpp::Time start_scan_time = this->now();
    bool status = drv->get_scanned_data(scan_vec, 150);
    if (!status) {
      delete drv;
      drv = switch_active_mode ? new pavo_driver(host_ip, host_port) : new pavo_driver();
      drv->pavo_open(lidar_ip, lidar_port);
      return;
    }

    rclcpp::Time end_scan_time = this->now();
    double scan_duration = (end_scan_time - start_scan_time).seconds();

    size_t node_count = scan_vec.size();
    int counts = node_count * ((angle_max - angle_min) / 270.0f);
    int angle_start = 135 + angle_min;
    int node_start = node_count * (angle_start / 270.0f);

    auto scanMsg = sensor_msgs::msg::LaserScan();
    scanMsg.header.stamp = start_scan_time;
    scanMsg.header.frame_id = frame_id;
    scanMsg.angle_min = DEG2RAD(angle_min);
    scanMsg.angle_max = DEG2RAD(angle_max);
    scanMsg.angle_increment = (scanMsg.angle_max - scanMsg.angle_min) / (double)counts;
    scanMsg.scan_time = scan_duration;
    scanMsg.time_increment = scan_duration / (double)node_count;
    scanMsg.range_min = range_min;
    scanMsg.range_max = range_max;

    scanMsg.ranges.resize(counts);
    scanMsg.intensities.resize(counts);

    for (int i = 0; i < counts; ++i) {
      float range = scan_vec[node_start].distance * 0.002;
      float intensity = scan_vec[node_start].intensity;
      if (range < range_min || range > range_max) {
        range = 0.0;
        intensity = 0.0;
      }

      if (!inverted) {
        scanMsg.ranges[i] = range;
        scanMsg.intensities[i] = intensity;
      } else {
        scanMsg.ranges[counts - 1 - i] = range;
        scanMsg.intensities[counts - 1 - i] = intensity;
      }
      node_start++;
    }

    scan_pub->publish(scanMsg);
  }

  std::string frame_id, lidar_ip, host_ip, scan_topic;
  int lidar_port, host_port, motor_speed, merge_coef, method;
  double angle_min, angle_max, range_min, range_max;
  bool inverted, enable_motor, switch_active_mode;

  bool initialized_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  pavo_driver *drv;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PavoScanNode>();

  if (!node->is_initialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Node failed to initialize.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

