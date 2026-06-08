#include "cam_streamer/cam_streamer.hpp"

CamStreamer::CamStreamer(const std::string& name) : Node(name)
{
  //capture
  {
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

    declare_parameter<std::string>("capture.device_id", "12345");
    capture.device_id = get_parameter("capture.device_id").as_string();

  }

  //gst
  {
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
    [this](const std_msgs::msg::String::SharedPtr msg)
    {
      gst.ip = msg->data;
      gst.ip_set = true;
    }
  );
  if (image.enable) image.publisher = create_publisher<quac_interfaces::msg::ImageBGRD>(
    std::string(get_name()) + "/bgrd", 
    rclcpp::QoS(2).best_effort().durability_volatile()
  );
  if (pointcloud.enable) pointcloud.publisher = create_publisher<sensor_msgs::msg::PointCloud2>(
    std::string(get_name()) + "/points", 
    rclcpp::QoS(2).best_effort().durability_volatile()
  );

  cam_info_publisher = create_publisher<sensor_msgs::msg::CameraInfo>(
    std::string(get_name()) + "/info", 
    2
  );

  RCLCPP_INFO(get_logger(), 
    "Started with the following configuration:\n"
    "  capture:\n"
    "    device_id: %s\n"
    "    fps: %d\n"
    "    color:\n"
    "      width: %d\n"
    "      height: %d\n"
    "    depth:\n"
    "      width: %d\n"
    "      height: %d\n"
    "  gst:\n"
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
    capture.device_id.c_str(), capture.fps, capture.color.width, capture.color.height, capture.depth.width, capture.depth.height,
    gst.port, gst.compression.c_str(), gst.jpeg.quality, gst.h264.bitrate, gst.h264.key_int_max,
    pointcloud.enable ? "true" : "false", pointcloud.interval, pointcloud.frame.c_str(),
    image.enable ? "true" : "false", image.interval, image.frame.c_str()
  );
}

void CamStreamer::push_gst_frame(void* data)
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
          "! jpegenc quality=" + std::to_string(gst.jpeg.quality) + " ! rtpjpegpay"
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
          "! x264enc speed-preset=ultrafast tune=zerolatency "
          " bitrate=" + std::to_string(gst.h264.bitrate) +
          " key-int-max=" + std::to_string(gst.h264.key_int_max) + " "
          "! rtph264pay pt=96 config-interval=1 "
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

  if (gst.ip_set)
  {
    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, 3 * capture.color.width * capture.color.height, nullptr);
    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_WRITE);
    memcpy(map.data, data, 3 * capture.color.width * capture.color.height);

    gst_buffer_unmap(buffer, &map);
    gst_app_src_push_buffer(GST_APP_SRC(gst.appsrc), buffer);
  }
}

void CamStreamer::deinit()
{
  if (gst.ip_set)
  {
    gst_element_set_state(gst.pipeline, GST_STATE_NULL);
    gst_object_unref(gst.pipeline);
  }
}