#define DEPTHAI_HAVE_OPENCV_SUPPORT
#include <opencv2/opencv.hpp>

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

    std::shared_ptr<dai::node::Camera> color_cam_node;
    dai::Node::Output* color_cam_out;

    std::shared_ptr<dai::node::Camera> left_cam_node;
    dai::Node::Output* left_cam_out;

    std::shared_ptr<dai::node::Camera> right_cam_node;
    dai::Node::Output* right_cam_out;

    std::shared_ptr<dai::node::StereoDepth> stereo_depth_node;
    std::shared_ptr<dai::node::Sync> sync_node;

    std::shared_ptr<dai::node::PointCloud> pointcloud_node;
    std::shared_ptr<dai::Device> device;
    std::shared_ptr<dai::MessageQueue> queue;
  } dai;

};