#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include "pavo_driver.h"

using namespace std::chrono_literals;

class PavoPcdNode : public rclcpp::Node {
public:
  PavoPcdNode() : Node("pavo_pcd_node") {
    // Declare and get parameters
    this->declare_parameter<std::string>("frame_id", "pcd_frame");
    this->declare_parameter<std::string>("cloud_topic", "cloud");
    this->declare_parameter<int>("motor_speed", 15);
    this->declare_parameter<int>("merge_coef", 2);
    this->declare_parameter<bool>("enable_motor", true);
    this->declare_parameter<std::string>("lidar_ip", "10.10.10.6");
    this->declare_parameter<int>("lidar_port", 2368);
    this->declare_parameter<std::string>("host_ip", "10.10.10.100");
    this->declare_parameter<int>("host_port", 2368);
    this->declare_parameter<int>("method", 0);
    this->declare_parameter<bool>("switch_active_mode", false);

    frame_id = this->get_parameter("frame_id").as_string();
    cloud_topic = this->get_parameter("cloud_topic").as_string();
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
    cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, 10);

    // Driver init
    drv = switch_active_mode ?
      new pavo::pavo_driver(host_ip, host_port) :
      new pavo::pavo_driver();

    if (!drv->pavo_open(lidar_ip, lidar_port)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open lidar connection.");
      rclcpp::shutdown();
      return;
    }

    drv->enable_motor(enable_motor);
    if (!enable_motor) {
      RCLCPP_WARN(this->get_logger(), "Motor disabled, shutting down.");
      rclcpp::shutdown();
      return;
    }

    if (method >= 0 && method <= 3) {
      drv->enable_tail_filter(method);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Invalid tail filter method.");
      rclcpp::shutdown();
      return;
    }

//    if (!drv->set_motor_speed(motor_speed)) {
  //    RCLCPP_ERROR(this->get_logger(), "Failed to set motor speed.");
    //  rclcpp::shutdown();
   //   return;
 //   }

    if (!drv->set_merge_coef(merge_coef)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set merge coefficient.");
      rclcpp::shutdown();
      return;
    }

    // Timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / motor_speed),
      std::bind(&PavoPcdNode::publish_cloud, this)
    );

    initialized_ = true;
  }

  bool is_initialized() const {
    return initialized_;
  }

  ~PavoPcdNode() {
    delete drv;
  }

private:
  void publish_cloud() {
    std::vector<pavo_response_pcd_t> pcd_vec;
    if (!drv->get_scanned_data(pcd_vec, 100)) return;

    int num_points = pcd_vec.size();
    if (num_points == 0) return;

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = this->now();
    cloud_msg.header.frame_id = frame_id;
    cloud_msg.height = 1;
    cloud_msg.width = num_points;
    cloud_msg.is_dense = false;

    //sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
    //modifier.setPointCloud2FieldsByString(2, "xyz", "intensity");
    //modifier.resize(num_points);

sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
modifier.setPointCloud2Fields(
  4,
  "x", 1, sensor_msgs::msg::PointField::FLOAT32,
  "y", 1, sensor_msgs::msg::PointField::FLOAT32,
  "z", 1, sensor_msgs::msg::PointField::FLOAT32,
  "intensity", 1, sensor_msgs::msg::PointField::FLOAT32
);
modifier.resize(num_points);


    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud_msg, "intensity");

    for (int i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
      *iter_x = pcd_vec[i].x * 0.002f;
      *iter_y = pcd_vec[i].y * 0.002f;
      *iter_z = pcd_vec[i].z;
      *iter_intensity = static_cast<float>(pcd_vec[i].intensity);
    }

    cloud_pub->publish(cloud_msg);
  }

  std::string frame_id, cloud_topic, lidar_ip, host_ip;
  int lidar_port, host_port, motor_speed, method, merge_coef;
  bool enable_motor, switch_active_mode;

  bool initialized_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
  rclcpp::TimerBase::SharedPtr timer_;
  pavo::pavo_driver *drv;
};

int main(int argc, char **argv) {
 rclcpp::init(argc, argv);
  auto node = std::make_shared<PavoPcdNode>();

  if (!node->is_initialized()) {
    RCLCPP_FATAL(rclcpp::get_logger("main"), "Node failed to initialize. Shutting down.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

