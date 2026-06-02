#include "realsense_streamer/realsense_streamer.hpp"
#include <librealsense2/h/rs_sensor.h>

std::atomic<bool> keep_running{true};

void handle_signal(int signum) {
  keep_running.store(false);
}

RealsenseStreamer::RealsenseStreamer() : Node("realsense_streamer")
{
  //capture
  {
    declare_parameter<std::string>("capture.serial_number", "12345");
    capture.serial_number = get_parameter("capture.serial_number").as_string();

    declare_parameter<int>("capture.color.width", 640);
    capture.color.width = get_parameter("capture.color.width").as_int();

    declare_parameter<int>("capture.color.height", 480);
    capture.color.height = get_parameter("capture.color.height").as_int();

    declare_parameter<int>("capture.depth.width", 640);
    capture.depth.width = get_parameter("capture.depth.width").as_int();

    declare_parameter<int>("capture.depth.height", 480);
    capture.depth.height = get_parameter("capture.depth.height").as_int();

    declare_parameter<int>("capture.fps", 30);
    capture.fps = get_parameter("capture.fps").as_int();
  }

  //gst
  {
    declare_parameter<int>("gst.width", 640);
    gst.width = get_parameter("gst.width").as_int();

    declare_parameter<int>("gst.height", 480);
    gst.height = get_parameter("gst.height").as_int();

    declare_parameter<int>("gst.port", 5000);
    gst.port = get_parameter("gst.port").as_int();

    declare_parameter<std::string>("gst.compression", "h264");
    gst.compression = get_parameter("gst.compression").as_string();

    //h264
    {
      declare_parameter<int>("gst.h264.bitrate", 5000);
      gst.h264.bitrate = get_parameter("gst.h264.bitrate").as_int();

      declare_parameter<int>("gst.h264.key_int_max", 10);
      gst.h264.key_int_max = get_parameter("gst.h264.key_int_max").as_int();
    }

    //jpeg
    {
      declare_parameter<int>("gst.jpeg.quality", 80);
      gst.jpeg.quality = get_parameter("gst.jpeg.quality").as_int();
    }

  }

  //pointcloud
  {
    declare_parameter<bool>("pointcloud.enable", true);
    pointcloud.enable = get_parameter("pointcloud.enable").as_bool();

    declare_parameter<std::string>("pointcloud.frame", "cam_frame");
    pointcloud.frame = get_parameter("pointcloud.frame").as_string();

    declare_parameter<int>("pointcloud.interval", 3);
    pointcloud.interval = get_parameter("pointcloud.interval").as_int();
    pointcloud.interval_i = 0;
  }

  //image
  {
    declare_parameter<bool>("image.enable", true);
    image.enable = get_parameter("image.enable").as_bool();

    declare_parameter<std::string>("image.frame", "cam_frame");
    image.frame = get_parameter("image.frame").as_string();

    declare_parameter<int>("image.interval", 3);
    image.interval = get_parameter("image.interval").as_int();
    image.interval_i = 0;
  }

  gst.ip_set = false;
  gst.ip_subscriber = create_subscription<std_msgs::msg::String>(
    "video_target_ip", 
    rclcpp::SensorDataQoS(), 
    std::bind(&RealsenseStreamer::ip_callback, this, std::placeholders::_1)
  );
  if (image.enable) image.publisher = create_publisher<quac_interfaces::msg::ImageBGRD>(
    std::string(get_name()) + "/bgrd", 
    rclcpp::QoS(2).best_effort().durability_volatile()
  );
  if (pointcloud.enable) pointcloud.publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    std::string(get_name()) + "/points", 
    rclcpp::QoS(2).best_effort().durability_volatile()
  );

  RCLCPP_INFO(get_logger(), 
    "Started with the following configuration:\n"
    "  capture:\n"
    "    serial_number: %s\n"
    "    fps: %d\n"
    "    color:\n"
    "      width: %d\n"
    "      height: %d\n"
    "    depth:\n"
    "      width: %d\n"
    "      height: %d\n"
    "  gst:\n"
    "    width: %d\n"
    "    height: %d\n"
    "    port: %d\n"
    "    compression: %s\n"
    "    jpeg:\n"
    "      quality: %d\n"
    "    h264:\n"
    "      bitrate: %d\n"
    "      key_int_max: %d\n"
    "  pointcloud:\n"
    "    enable: %s\n"
    "    interval: %d\n"
    "    frame: %s\n"
    "  image:\n"
    "    enable: %s\n"
    "    interval: %d\n"
    "    frame: %s\n",
    capture.serial_number.c_str(), capture.fps, capture.color.width, capture.color.height, capture.depth.width, capture.depth.height,
    gst.width, gst.height, gst.port, gst.compression.c_str(), gst.jpeg.quality, gst.h264.bitrate, gst.h264.key_int_max,
    pointcloud.enable ? "true" : "false", pointcloud.interval, pointcloud.frame.c_str(),
    image.enable ? "true" : "false", image.interval, image.frame.c_str()
  );
}

bool camera_is_connected(const std::string& serial_number) {
  rs2::context ctx;
  rs2::device_list devices = ctx.query_devices();

  for (auto&& dev : devices)
    if (dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) == serial_number)
      return true;
    
  return false;
}

void RealsenseStreamer::ip_callback(const std_msgs::msg::String::SharedPtr msg)
{
  gst.ip = msg->data;
  gst.ip_set = true;
}

void RealsenseStreamer::pointcloud_loop()
{
  rs2::align align_to_depth(RS2_STREAM_DEPTH);

  while (keep_running.load())
  {
    pointcloud.mutex.lock();
    if (pointcloud.available) pointcloud.working = true;
    pointcloud.mutex.unlock();

    if (pointcloud.working)
    {
      rs2::frameset aligned_frames = align_to_depth.process(pointcloud.frameset);

      rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
      rs2::video_frame color_frame = aligned_frames.get_color_frame();

      pointcloud.points = pointcloud.pc.calculate(depth_frame);
      pointcloud.pc.map_to(color_frame);

      auto vertices = pointcloud.points.get_vertices();
      auto tex_coords = pointcloud.points.get_texture_coordinates();

      const uint8_t* color_data = static_cast<const uint8_t*>(color_frame.get_data());

      size_t num_points = pointcloud.points.size();

      pointcloud.msg.header.stamp = now();
      pointcloud.msg.header.frame_id = pointcloud.frame;

      pointcloud.msg.height = 1;
      pointcloud.msg.width = num_points;
      pointcloud.msg.is_dense = false;
      pointcloud.msg.is_bigendian = false;

      sensor_msgs::PointCloud2Modifier modifier(pointcloud.msg);
      modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      modifier.resize(num_points);

      sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud.msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud.msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud.msg, "z");

      sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(pointcloud.msg, "r");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(pointcloud.msg, "g");
      sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(pointcloud.msg, "b");

      for (size_t i = 0; i < num_points; ++i)
      {
        const rs2::vertex& v = vertices[i];
        const rs2::texture_coordinate& t = tex_coords[i];

        if (v.z <= 0.f || !std::isfinite(v.z))
        {
          *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
          *iter_r = *iter_g = *iter_b = 0;
        }
        else
        {
          *iter_x = v.x;
          *iter_y = v.y;
          *iter_z = v.z;

          int u = std::min(std::max(int(t.u * (float)capture.depth.width), 0), capture.depth.width - 1);
          int v_px = std::min(std::max(int(t.v * (float)capture.depth.height), 0), capture.depth.height - 1);

          int idx = (v_px * capture.depth.width + u) * 3;

          *iter_b = color_data[idx + 0];
          *iter_g = color_data[idx + 1];
          *iter_r = color_data[idx + 2];
        }

        ++iter_x; ++iter_y; ++iter_z;
        ++iter_r; ++iter_g; ++iter_b;
      }

      pointcloud.publisher->publish(pointcloud.msg);
      pointcloud.mutex.lock();
      pointcloud.working = false;
      pointcloud.available = false;
      pointcloud.mutex.unlock();
    }
    else std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void RealsenseStreamer::run()
{
  // make sure camera is connected
  if (camera_is_connected(capture.serial_number) == false)
  {
    RCLCPP_ERROR(get_logger(), "camera with serial %s not connected", capture.serial_number.c_str());
    return;
  }

  capture.cfg.enable_device(capture.serial_number);
  capture.cfg.enable_stream(RS2_STREAM_COLOR, capture.color.width, capture.color.height, RS2_FORMAT_BGR8, capture.fps);
  capture.cfg.enable_stream(RS2_STREAM_DEPTH, capture.depth.width, capture.depth.height, RS2_FORMAT_Z16, capture.fps);

  rs2::align align_to_color(RS2_STREAM_COLOR);

  capture.pipeline.start(capture.cfg);

  signal(SIGTERM, handle_signal);
  signal(SIGINT, handle_signal);
  if (pointcloud.enable)
  {
    pointcloud.working = false;
    pointcloud.available = false;
    pointcloud.thread = std::thread([this](){ pointcloud_loop();});
  }

  RCLCPP_INFO(get_logger(), "Camera opened");
  while (keep_running.load())
  {
    if (gst.ip_set == false)
    {
      rclcpp::spin_some(shared_from_this());
      
      if (gst.ip_set)
      {
        gst_init(NULL, NULL);

        std::string pipeline_desc;

        if (gst.compression == "jpeg")
        {
          pipeline_desc =
            "appsrc name=appsrc format=time "
            "caps=video/x-raw,format=BGR,width=" + std::to_string(capture.color.width) +
            ",height=" + std::to_string(capture.color.height) +
            ",framerate=" + std::to_string(capture.fps) + "/1 "
            "! videoconvert "
            "! videoscale "
            "! video/x-raw,width=" + std::to_string(gst.width) + ",height=" + std::to_string(gst.height) + " "
            "jpegenc quality=" + std::to_string(gst.jpeg.quality) + " ! rtpjpegpay"
            "! udpsink host=" + gst.ip +
            " port=" + std::to_string(gst.port) + " sync=false";  
        }
        else if (gst.compression == "h264")
        {
          pipeline_desc =
            "appsrc name=appsrc format=time "
            "caps=video/x-raw,format=BGR,width=" + std::to_string(capture.color.width) +
            ",height=" + std::to_string(capture.color.height) +
            ",framerate=" + std::to_string(capture.fps) + "/1 "
            "! videoconvert "
            "! videoscale "
            "! video/x-raw,width=" + std::to_string(gst.width) + ",height=" + std::to_string(gst.height) + " "
            "! x264enc speed-preset=ultrafast tune=zerolatency "
            "bitrate=" + std::to_string(gst.h264.bitrate) +
            " key-int-max=" + std::to_string(gst.h264.key_int_max) + " "
            "! rtph264pay config-interval=1 "
            "! udpsink host=" + gst.ip +
            " port=" + std::to_string(gst.port) + " sync=false";
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "unsupported compression %s", gst.compression.c_str());
          return;
        }

        GError *error = nullptr;
        gst.pipeline = gst_parse_launch(pipeline_desc.c_str(), &error);

        if (!gst.pipeline || error)
        {
          RCLCPP_ERROR(get_logger(), "Failed to create Gstreamer pipeline %s", pipeline_desc.c_str());
          if (error)
          {
            RCLCPP_ERROR(get_logger(), "GError: %s", error->message);
            g_error_free(error);
          }
          return;
        }

        gst.appsrc = gst_bin_get_by_name(GST_BIN(gst.pipeline), "appsrc");
        gst_element_set_state(gst.pipeline, GST_STATE_PLAYING);

        RCLCPP_INFO(get_logger(), "Streaming camera on %s:%d", gst.ip.c_str(), gst.port);
        gst.ip_subscriber.reset();
      }
    } 

    rs2::frameset frames = capture.pipeline.wait_for_frames();
    rs2::video_frame color_frame = frames.get_color_frame();
    rs2::depth_frame depth_frame = frames.get_depth_frame();

    if (gst.ip_set)
    {
      GstBuffer *buffer = gst_buffer_new_allocate(nullptr, 3 * capture.color.width * capture.color.height, nullptr);
      GstMapInfo map;
      gst_buffer_map(buffer, &map, GST_MAP_WRITE);
      memcpy(map.data, color_frame.get_data(), 3 * capture.color.width * capture.color.height);

      gst_buffer_unmap(buffer, &map);
      gst_app_src_push_buffer(GST_APP_SRC(gst.appsrc), buffer);
    }

    // image
    if (image.interval_i == 0 && image.enable)
    {
      rs2::frameset aligned_frames = align_to_color.process(frames);
      rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();

      if (!aligned_depth_frame) continue;

      image.msg.header.stamp = now();
      image.msg.header.frame_id = image.frame;

      image.msg.height = capture.color.height;
      image.msg.width = capture.color.width;
      image.msg.depth_scale = aligned_depth_frame.get_units();

      rs2::video_stream_profile color_profile = 
      color_frame.get_profile().as<rs2::video_stream_profile>();
      rs2_intrinsics intr = color_profile.get_intrinsics();

      image.msg.fx = intr.fx;
      image.msg.fy = intr.fy;
      image.msg.ppx = intr.ppx;
      image.msg.ppy = intr.ppy;

      image.msg.bgr_data.resize(capture.color.width * capture.color.height * 3);
      memcpy(image.msg.bgr_data.data(), color_frame.get_data(), capture.color.width * capture.color.height * 3);

      image.msg.depth_data.resize(capture.color.width * capture.color.height);
      memcpy(image.msg.depth_data.data(), aligned_depth_frame.get_data(), capture.color.width * capture.color.height * 2);

      image.publisher->publish(image.msg);
    }

    if (image.enable) image.interval_i = (image.interval_i + 1) % image.interval;

    // pointcloud
    if (pointcloud.enable) pointcloud.interval_i++;
    if (pointcloud.interval_i >= pointcloud.interval && pointcloud.enable)
    {
      pointcloud.mutex.lock();
      if (pointcloud.working == false)
      {
        pointcloud.available = true;
        pointcloud.frameset = frames;
        pointcloud.interval_i = 0;
      }
      pointcloud.mutex.unlock();
    }
  }

  if (pointcloud.enable) pointcloud.thread.join();
  capture.pipeline.stop();
  RCLCPP_INFO(get_logger(), "Closed camera");

  if (gst.ip_set)
  {
    gst_element_set_state(gst.pipeline, GST_STATE_NULL);
    gst_object_unref(gst.pipeline);
  }
}

int main (int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RealsenseStreamer>();
  node->run();
  
  rclcpp::shutdown();
}