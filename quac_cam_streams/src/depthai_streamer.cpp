#include "depthai_streamer/depthai_streamer.hpp"

std::atomic<bool> keep_running{true};
void handle_signal(int signum) { keep_running.store(false); }

DepthaiStreamer::DepthaiStreamer() : CamStreamer("depthai_streamer") {}

bool camera_is_connected(const std::string& id)
{
  auto devices = dai::Device::getAllConnectedDevices();
  printf("%d device connected \n", devices.size()); fflush(stdout);
  for(const auto& deviceInfo : devices)
    if(deviceInfo.getDeviceId() == id) return true;

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

  dai.device = std::make_shared<dai::Device>(dai::DeviceInfo(capture.device_id));
  dai.pipeline = std::make_shared<dai::Pipeline>(dai.device);

  dai.color_cam_node = dai.pipeline->create<dai::node::Camera>();
  dai.color_cam_node->build(dai::CameraBoardSocket::CAM_A);

  dai.left_cam_node = dai.pipeline->create<dai::node::Camera>();
  dai.left_cam_node->build(dai::CameraBoardSocket::CAM_B);

  dai.right_cam_node = dai.pipeline->create<dai::node::Camera>();
  dai.right_cam_node->build(dai::CameraBoardSocket::CAM_C);

  dai.stereo_depth_node = dai.pipeline->create<dai::node::StereoDepth>();
  dai.stereo_depth_node->setExtendedDisparity(true);
  dai.stereo_depth_node->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::FAST_ACCURACY);

  dai.sync_node = dai.pipeline->create<dai::node::Sync>();
  dai.sync_node->setSyncThreshold(std::chrono::duration<int64_t, std::nano>(static_cast<int64_t>(1e9 / (4.0 * (double)capture.fps))));

  dai.color_cam_out = dai.color_cam_node->requestOutput(std::make_pair(capture.color.width, capture.color.height), dai::ImgFrame::Type::BGR888i, dai::ImgResizeMode::CROP, capture.fps, true);
  dai.left_cam_out = dai.left_cam_node->requestOutput(std::make_pair(capture.depth.width, capture.depth.height), std::nullopt, dai::ImgResizeMode::CROP, capture.fps);
  dai.right_cam_out = dai.right_cam_node->requestOutput(std::make_pair(capture.depth.width, capture.depth.height), std::nullopt, dai::ImgResizeMode::CROP, capture.fps);

  dai.color_cam_out->link(dai.sync_node->inputs["rgb"]);
  dai.left_cam_out->link(dai.stereo_depth_node->left);
  dai.right_cam_out->link(dai.stereo_depth_node->right);

  dai.stereo_depth_node->depth.link(dai.sync_node->inputs["depth_aligned"]);
  dai.color_cam_out->link(dai.stereo_depth_node->inputAlignTo);

  dai.queue = dai.sync_node->out.createOutputQueue(1, false);
  
  
  auto calib_data = dai.device->readCalibration();
  auto intrinsics = calib_data.getCameraIntrinsics(
    dai::CameraBoardSocket::CAM_A, 
    capture.color.width, 
    capture.color.height
  );
  

  dai.pipeline->start();

  signal(SIGTERM, handle_signal);
  signal(SIGINT, handle_signal);


  RCLCPP_INFO(get_logger(), "Camera opened");
  
  while (keep_running.load() && dai.pipeline->isRunning())
  {
    auto message_group = dai.queue->get<dai::MessageGroup>();

    auto image_color_frame = message_group->get<dai::ImgFrame>("rgb");
    auto image_depth_frame = message_group->get<dai::ImgFrame>("depth_aligned");

    if(image_depth_frame != nullptr)
    {
      cv::Mat color_frame = image_color_frame->getFrame();
      cv::Mat depth_frame = image_depth_frame->getFrame();

      push_gst_frame(color_frame.data);

      sensor_msgs::msg::CameraInfo cam_info;
      cam_info.header.stamp = now();
      cam_info.header.frame_id = image.frame;
      cam_info.width = capture.color.width;
      cam_info.height = capture.color.height;
      cam_info.k[0] = intrinsics[0][0];
      cam_info.k[2] = intrinsics[0][2];
      cam_info.k[4] = intrinsics[1][1];
      cam_info.k[5] = intrinsics[1][2];

      cam_info_publisher->publish(cam_info);

      // image
      if (image.interval_i == 0 && image.enable)
      {
        image.msg.header = cam_info.header;

        image.msg.height = capture.color.height;
        image.msg.width = capture.color.width;
        image.msg.depth_scale = 0.001;

        image.msg.fx = 100;
        image.msg.fy = 100;
        image.msg.ppx = 100;
        image.msg.ppy = 100;

        image.msg.bgr_data.resize(capture.color.width * capture.color.height * 3);
        memcpy(image.msg.bgr_data.data(), color_frame.data, capture.color.width * capture.color.height * 3);

        image.msg.depth_data.resize(capture.color.width * capture.color.height);
        memcpy(image.msg.depth_data.data(), depth_frame.data, capture.color.width * capture.color.height * 2);

        image.publisher->publish(image.msg);
      }
      if (image.enable) image.interval_i = (image.interval_i + 1) % image.interval;

    }
  }

  dai.pipeline->stop();
  dai.pipeline->wait();

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