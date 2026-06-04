#include <cam_streamer/cam_streamer.hpp>
#include <librealsense2/rs.hpp>
#include <thread>
#include <mutex>

class RealsenseStreamer : public CamStreamer
{
public:
  RealsenseStreamer();

  void ip_callback(const std_msgs::msg::String::SharedPtr msg);
  void run();

  void pointcloud_loop();


  struct
  {
    struct
    {
      rs2::config cfg;
      rs2::pipeline pipeline;
    } capture;

    struct
    {
      std::thread thread;
      
      std::mutex mutex;
      rs2::frameset frameset;
      bool available;
      bool working;

      rs2::pointcloud pc;
      rs2::points points;
    } pointcloud;
  } rs;

};