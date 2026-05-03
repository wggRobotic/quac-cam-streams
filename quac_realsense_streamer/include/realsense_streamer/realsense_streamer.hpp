#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>
#include <std_msgs/msg/string.hpp>
#include <quac_interfaces/msg/image_bgrd.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <thread>
#include <mutex>

class RealsenseStreamer : public rclcpp::Node
{
public:
  RealsenseStreamer();

  void ip_callback(const std_msgs::msg::String::SharedPtr msg);
  void run();

  void pointcloud_loop();

  struct
  {
    rs2::config cfg;
    rs2::pipeline pipeline;
    std::string serial_number;
    struct
    {
      int width, height;
    } color;
    struct
    {
      int width, height;
    } depth;
    int fps;
  } capture;

  struct
  {
    GstElement* pipeline;
    GstElement* appsrc;
    int width, height, port;
    std::string ip;
    bool ip_set;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ip_subscriber;
    std::string compression;

    struct
    {
      int quality;
    } jpeg;
    
    struct
    {
      int bitrate;
      int key_int_max;
    } h264;
  } gst;

  struct
  {
    std::string frame;

    std::thread thread;
    
    std::mutex mutex;
    rs2::frameset frameset;
    bool available;
    bool working;

    int interval;
    int interval_i;

    rs2::pointcloud pc;
    rs2::points points;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    sensor_msgs::msg::PointCloud2 msg;
  } pointcloud;

  struct
  {
    std::string frame;

    int interval;
    int interval_i;

    rclcpp::Publisher<quac_interfaces::msg::ImageBGRD>::SharedPtr publisher;
    quac_interfaces::msg::ImageBGRD msg;
  } image;

};