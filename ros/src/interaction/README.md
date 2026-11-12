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

## Wristband

- Topic: `/wristband/vibration` using `std_msgs/msg/UInt8MultiArray`.
- Data: `[side, magnitude]`, where side is `0` for left, `1` for right, `2` for forward or `3` for back and magnitude is 0 to 255.
- BLE output: `<l:180>`, `<r:180>`, `<f:180>` or `<b:180>`. Each message sends one frame.
- ESP32 BLE device name: `naviGoatWristband`.

Mount the host Bluetooth D-Bus socket when starting the container:

```bash
docker run --rm -it --name navigoat-dev --network=host -v /run/dbus:/run/dbus -e DISPLAY -v "$PWD:/workspace" -v /tmp/.X11-unix:/tmp/.X11-unix:ro navigoat:dev bash
```

Inside the container, from `/workspace`:

```bash
colcon build --packages-select wristband_nodes --symlink-install
source install/setup.bash
ros2 launch wristband_nodes wristband_launch.py
```
