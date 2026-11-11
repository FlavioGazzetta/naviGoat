# Camera: Orbbec Femto Bolt

[camera.py](camera.py) is a ROS 2 node that reads the Femto Bolt through the Orbbec SDK (`pyorbbecsdk2`, in the `PETE=true` image) and publishes:

| Topic | Type | |
| --- | --- | --- |
| `/camera/color/image_raw` | `sensor_msgs/Image` | colour, `bgr8`, 1280×720 at about 18 fps on the Jetson |
| `/camera/depth_registered/image_rect` | `sensor_msgs/Image` | depth aligned to colour, `16UC1` in mm, 0 = no reading |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | colour intrinsics |
| `/camera/color/image_raw/compressed` | `sensor_msgs/CompressedImage` | small 640 px jpeg preview, for watching from another computer |

All are best effort. Colour and depth of a frame share the same timestamp. Perception ([autonomy/perception](../../autonomy/perception)) and the people tracker both use these topics. Only one program can open the camera, so run either this node or the Orbbec ROS 2 driver, not both.

## Run

The container needs the USB camera and the GPU (the SDK's depth engine runs on it):

```bash
docker run --rm -it --network=host --ipc=host --runtime nvidia --gpus all \
  --privileged -v /dev:/dev -v "$PWD:/workspace" navigoat:dev bash
ros2 launch /workspace/ros/src/hardware/camera/launch/camera.launch.py
```

Or start it together with perception: `ros2 launch /workspace/ros/src/autonomy/perception/launch/perception.launch.py camera:=true`.

## Watch it

On a laptop on the same network (it needs ROS 2 Jazzy and OpenCV), `python3 view_camera.py` opens a window with the preview. Press `q` to quit. Full-size frames don't get through a USB or Wi-Fi link, which is why it shows the jpeg preview.
