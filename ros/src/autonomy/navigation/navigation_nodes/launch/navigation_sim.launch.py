"""The whole navigation pipeline in Gazebo: Gazebo, RTAB-Map, Nav2 and RViz (from simulation.launch.py), plus the
navigation nodes.

Map first (mode:=mapping), then navigate on the saved map:

    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py mode:=mapping
    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py

With made-up people to test the social layer (edit the file to put them where your map has room):

    ros2 launch .../navigation_sim.launch.py fake_people:=/workspace/ros/src/autonomy/navigation/config/fake_people.yaml

With REAL people, seen by Enid's tracker and the real camera (on the Jetson), put on the simulated map in front of the
simulated robot:

    ros2 launch .../navigation_sim.launch.py real_people:=true

The simulation then runs on ROS domain sim_domain (42), so the real camera's images don't mix with Gazebo's: in the
other terminals, do `export ROS_DOMAIN_ID=42` before using ros2. The tracker stays on its domain (tracker_domain, 0).

The places are kept in ros/src/simulation/maps/<world>_places.yaml, next to <world>.db, the map they belong to.
"""

# the full simulation in one command: the team's simulation.launch.py (Gazebo + RTAB-Map + Nav2 + RViz),
# plus our navigation nodes from navigation.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))  # navigation_nodes/launch
SRC_DIR = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))  # ros/src


def launch_navigation(context):
    # we need the world's name as normal text to make the places file name ("indoor.sdf" -> "indoor_places.yaml").
    # launch arguments only turn into text inside an OpaqueFunction, that's why this is a separate function
    world = os.path.splitext(os.path.basename(LaunchConfiguration('world').perform(context)))[0]
    # with real people, people_node listens to the tracker on its own domain (-1 = on ours, like normally)
    real_people = LaunchConfiguration('real_people').perform(context) == 'true'
    tracker_domain = LaunchConfiguration('tracker_domain').perform(context) if real_people else '-1'
    # start our nodes (navigation.launch.py) with these arguments
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'navigation.launch.py')),
        launch_arguments={
            'places_file': os.path.join(SRC_DIR, 'simulation', 'maps', f'{world}_places.yaml'),
            'use_sim_time': 'true',  # in Gazebo everything runs on the simulation's clock
            'fake_people': LaunchConfiguration('fake_people'),
            'tracker_domain': tracker_domain,
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        # Localisation by default: Nav2 needs a finished map, and saved places only make sense on a map that stays the same.
        DeclareLaunchArgument('mode', default_value='localisation', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('world', default_value='indoor.sdf'),
        # the real robot's RViz (autonomy/navigation/config/robot.rviz) by default, so the sim looks like the robot
        # on the Jetson over SSH (no screen): headless:=true rviz:=false, and RViz on the laptop (navigation_scenarios.md)
        DeclareLaunchArgument('headless', default_value='false', choices=['true', 'false'],
                              description='true on a machine without a screen: Gazebo server only, no window'),
        DeclareLaunchArgument('rviz', default_value='true', choices=['true', 'false'],
                              description='false to run RViz on another computer instead'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(SRC_DIR, 'autonomy', 'navigation', 'config', 'robot.rviz'),
                              description='RViz config file'),
        DeclareLaunchArgument('fake_people', default_value='none', description='YAML file of made-up people, for testing'),
        DeclareLaunchArgument('real_people', default_value='false', choices=['true', 'false'],
                              description="real people from Enid's tracker and the real camera, on the simulated map"),
        DeclareLaunchArgument('sim_domain', default_value='42', description='ROS domain of the simulation, with real_people'),
        DeclareLaunchArgument('tracker_domain', default_value='0', description="ROS domain of Enid's tracker, with real_people"),
        # with real people, the real camera and the simulated one both publish /camera/color/image_raw (and the depth
        # and camera_info), so RTAB-Map and the tracker would get both. to keep them apart, everything this file starts
        # (Gazebo's bridge, RTAB-Map, Nav2, RViz, our nodes) goes on its own ROS domain, which is like a separate network:
        # nodes only see the nodes on the same domain. people_node is the one exception: it also listens to the tracker's
        # domain (relay_tracks_from() in people_node.py). this has to come first, so everything started after it gets it
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('sim_domain'),
                               condition=IfCondition(LaunchConfiguration('real_people'))),
        LogInfo(msg=['Real people: the simulation is on ROS domain ', LaunchConfiguration('sim_domain'),
                     '. In other terminals: export ROS_DOMAIN_ID=', LaunchConfiguration('sim_domain')],
                condition=IfCondition(LaunchConfiguration('real_people'))),
        # Gazebo, RTAB-Map, Nav2 and RViz, from the simulation folder
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'simulation', 'launch', 'simulation.launch.py')),
            launch_arguments={'mode': LaunchConfiguration('mode'), 'world': LaunchConfiguration('world'),
                              'rviz_config': LaunchConfiguration('rviz_config'),
                              'headless': LaunchConfiguration('headless'), 'rviz': LaunchConfiguration('rviz')}.items(),
        ),
        # and our nodes
        OpaqueFunction(function=launch_navigation),
    ])
