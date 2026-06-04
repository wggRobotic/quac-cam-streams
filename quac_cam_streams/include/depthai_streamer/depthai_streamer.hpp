#include <cam_streamer/cam_streamer.hpp>
#include <depthai/depthai.hpp>
#include <thread>
#include <mutex>

class DepthaiStreamer : public CamStreamer
{
public:
  DepthaiStreamer();

  void run();

  struct
  {
    dai::Pipeline pipeline;
    std::shared_ptr<dai::node::ColorCamera> color_cam_node;
    std::shared_ptr<dai::node::MonoCamera> mono_cam_left_node;
    std::shared_ptr<dai::node::MonoCamera> mono_cam_right_node;
    std::shared_ptr<dai::node::StereoDepth> stereo_depth_node;
    std::shared_ptr<dai::node::PointCloud> pointcloud_node;
    std::shared_ptr<dai::node::XLinkOut> rgb_link_node;
    std::shared_ptr<dai::node::XLinkOut> depth_link_node;
    std::shared_ptr<dai::node::XLinkOut> pointcloud_link_node;
    dai::Device device;
    std::shared_ptr<dai::DataOutputQueue> rgb_queue;
    std::shared_ptr<dai::DataOutputQueue> depth_queue;
    std::shared_ptr<dai::DataOutputQueue> pointcloud_queue;
  } dai;

};