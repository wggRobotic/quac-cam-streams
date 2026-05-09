# quac-realsense-streamer
quac_realsense_streamer package

## realsense_streamer

Streams video over udp with gstreamer and high resolution bgrd images and pointcloud with ros2

### Subscriber
- `video_target_ip` : `std_msgs/msg/String` until Gstreamer starts
### Publishers
- `<_node_name>/bgrd` : `quac_interfaces/msg/ImageBGRD`
- `<_node_name>/points` : `sensor_msgs/msg/PointCloud2`

### Parameters

```
capture:
  serial_number: String       # realsense camera serial number

  fps: int                    # video fps
  color:
    width: int                # color video width
    height: int               # color video height
  depth:
    width: int                # depth video width
    height: int               # depth video height

gst:
  port: int                   # udp port to which gstreamer streams
  width: int                  # (scaled) width of the gstreamer stream
  height: int                 # (scaled) height of the gstreamer stream

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
