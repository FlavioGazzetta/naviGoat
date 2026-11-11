# Interaction

## Handle

- Topic: `/handle/vibration` using `std_msgs/msg/UInt8MultiArray`.
- Data: `[side, magnitude]`, where side is `0` for left or `1` for right and magnitude is 0 to 255. For example, `[1, 180]` vibrates the right side.
- Serial output: `<r:180>\n` at 115200 baud for that example. Each message sends one frame.
- Device: `/dev/handle` inside the container, mapped from the ESP32 at `/dev/ttyUSB0` on the Jetson.

From the repository root:

```bash
docker run --rm -it --name navigoat-dev --network=host --device=/dev/ttyUSB0:/dev/handle -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Inside the container, from `/workspace`:

```bash
colcon build --packages-select handle_nodes --symlink-install
source install/setup.bash
ros2 launch handle_nodes handle_launch.py
```
