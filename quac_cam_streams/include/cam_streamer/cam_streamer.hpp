#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <quac_interfaces/msg/image_bgrd.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

class CamStreamer : public rclcpp::Node
{
public:
  CamStreamer(const std::string& name);
  
  void deinit();
  void push_gst_frame(void* data);

  struct
  {
    struct
    {
      int width, height;
    } color;
    struct
    {
      int width, height;
    } depth;
    int fps;
    std::string device_id;
  } capture;

  struct
  {
    GstElement* pipeline;
    GstElement* appsrc;
    int port;
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
    bool enable;
    std::string frame;

    int interval;
    int interval_i;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    sensor_msgs::msg::PointCloud2 msg;
  } pointcloud;

  struct
  {
    bool enable;
    std::string frame;

    int interval;
    int interval_i;

    rclcpp::Publisher<quac_interfaces::msg::ImageBGRD>::SharedPtr publisher;
    quac_interfaces::msg::ImageBGRD msg;
  } image;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_publisher;
};