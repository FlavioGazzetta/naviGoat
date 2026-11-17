cmake_minimum_required(VERSION 3.8)
project(navigation_resources)

find_package(ament_cmake REQUIRED)

install(FILES ../config/nav2_params.yaml ../config/navigation_hardware.rviz DESTINATION share/${PROJECT_NAME}/navigation/config)
install(FILES ../../slam/launch/mapping.launch.py ../../slam/launch/slam.launch.py ../../slam/launch/lidar_test.launch.py DESTINATION share/${PROJECT_NAME}/autonomy/slam/launch)
install(FILES ../../slam/config/rtabmap.yaml ../../slam/config/rtabmap_hardware.yaml ../../slam/config/mapping.rviz ../../slam/config/icp.yaml DESTINATION share/${PROJECT_NAME}/autonomy/slam/config)
install(FILES ../../../bringup/description/pioneer3at_lidar.xacro DESTINATION share/${PROJECT_NAME}/bringup/description)
install(FILES ../../../hardware/camera/launch/camera.launch.py DESTINATION share/${PROJECT_NAME}/hardware/camera/launch)
install(FILES ../../../hardware/camera/camera.py DESTINATION share/${PROJECT_NAME}/hardware/camera)

ament_package()
