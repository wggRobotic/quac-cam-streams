# quac-cam-streams
quac_cam_streams package

## cam_streamer

Streams video over udp with gstreamer and high resolution bgrd images and pointcloud with ros2

### Subscriber
- `video_target_ip` : `std_msgs/msg/String` until Gstreamer starts
### Publishers
- `<_node_name>/bgrd` : `quac_interfaces/msg/ImageBGRD`
- `<_node_name>/points` : `sensor_msgs/msg/PointCloud2`

### Parameters

```
capture:
  device_id: String           # camera device id

  fps: int                    # video fps
  color:
    width: int                # color video width
    height: int               # color video height
  depth:
    width: int                # depth video width
    height: int               # depth video height

gst:
  port: int                   # udp port to which gstreamer streams
  compression: String         # compression type, either "jpeg" or "h264" 
  jpeg:
    quality: int              # jpeg quality in percent
  h264:
    bitrate: int              # bitrate in kb
    key_int_max: int          # interval in which key frame is sent

pointcloud:
  enable: bool                # whether to enable pointcloud publishing
  interval: int               # interval in all frames in which pointcloud is published
  frame: String               # tf frame of depth camera
image:
  enable: bool                # whether to enable bgrd image publishing
  interval: int               # interval in all frames in which bgrd image is published
  frame: String               # tf frame of color camera
```

## realsense_streamer : cam_streamer
## depthai_streamer : cam_streamer