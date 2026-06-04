#include "depthai_streamer/depthai_streamer.hpp"

std::atomic<bool> keep_running{true};
void handle_signal(int signum) { keep_running.store(false); }

DepthaiStreamer::DepthaiStreamer() : CamStreamer("depthai_streamer") {}

bool camera_is_connected(const std::string& id)
{
  std::vector<dai::DeviceInfo> devices = dai::Device::getAllAvailableDevices();

  for(const auto& deviceInfo : devices) if(deviceInfo.getMxId() == id) return true;

  return false;
}

void DepthaiStreamer::run()
{
  // make sure camera is connected
  if (camera_is_connected(capture.device_id) == false)
  {
    RCLCPP_ERROR(get_logger(), "camera with mxID %s not connected", capture.device_id.c_str());
    return;
  }

  dai.color_cam_node = dai.pipeline.create<dai::node::ColorCamera>();
  dai.color_cam_node->setBoardSocket(dai::CameraBoardSocket::CAM_A);
  dai.color_cam_node->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  dai.color_cam_node->setPreviewSize(640, 480);

  dai.mono_cam_left_node = dai.pipeline.create<dai::node::MonoCamera>();
  dai.mono_cam_left_node->setCamera(dai::CameraBoardSocket::LEFT);
  dai.mono_cam_left_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
  
  dai.mono_cam_right_node = dai.pipeline.create<dai::node::MonoCamera>();
  dai.mono_cam_right_node->setCamera(dai::CameraBoardSocket::RIGHT);
  dai.mono_cam_right_node->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

  dai.stereo_depth_node = dai.pipeline.create<dai::node::StereoDepth>();
  dai.stereo_depth_node->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  dai.stereo_depth_node->setDepthAlign(dai::CameraBoardSocket::CAM_A);
  dai.stereo_depth_node.setDefaultProfilePreset(HIGH_DENSITY);

  dai.mono_cam_left_node->out.link(dai.stereo_depth_node->left);
  dai.mono_cam_right_node->out.link(dai.stereo_depth_node->right);

  dai.rgb_link_node = dai.pipeline.create<dai::node::XLinkOut>();
  dai.rgb_link_node->setStreamName("rgb");
  dai.color_cam_node->preview.link(dai.rgb_link_node->input);

  dai.depth_link_node = dai.pipeline.create<dai::node::XLinkOut>();
  dai.depth_link_node->setStreamName("depth");
  dai.stereo_depth_node->depth.link(dai.depth_link_node->input);

  if (pointcloud.enable)
  {
    dai.pointcloud_node = dai.pipeline.create<dai::node::PointCloud>();

    dai.stereo_depth_node->depth.link(dai.pointcloud_node->inputDepth);
    dai.color_cam_node->preview.link(dai.pointcloud_node->inputTexture);

    dai.pointcloud_link_node = dai.pipeline.create<dai::node::XLinkOut>();
    dai.pointcloud_link_node->setStreamName("pcl");
    dai.pointcloud_node->outputPointCloud.link(dai.pointcloud_link_node->input);
  }

  dai.device = dai::Device(dai.pipeline, dai::DeviceInfo(mxId));

  dai.rgb_queue = dai.device.getOutputQueue("rgb");
  dai.depth_queue = dai.device.getOutputQueue("depth");
  if (pointcloud.enable) dai.pointcloud_queue = dai.device.getOutputQueue("pcl");

  signal(SIGTERM, handle_signal);
  signal(SIGINT, handle_signal);


  RCLCPP_INFO(get_logger(), "Camera opened");
  
  while (keep_running.load())
  {

    // image
    if (image.interval_i == 0 && image.enable)
    {
    }

    if (image.enable) image.interval_i = (image.interval_i + 1) % image.interval;

  }

  RCLCPP_INFO(get_logger(), "Closed camera");

  deinit();
}

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DepthaiStreamer>();
  node->run();
  
  rclcpp::shutdown();
}