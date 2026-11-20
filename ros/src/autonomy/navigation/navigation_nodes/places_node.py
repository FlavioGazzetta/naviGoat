"""Drives the robot with Nav2: to places saved on the map, to objects perception finds, and to points.
Started by launch/navigation.launch.py. Its topics are listed in autonomy/navigation/README.md.

The saved places are kept in a YAML file (places_file) that looks like:

    places:
      kitchen: {x: 1.25, y: -3.4, yaw: 1.571}

They only make sense on the map they were saved on: keep the file next to that map, and start again if it is rebuilt.

Only a goal's position matters: the robot arrives in whatever direction is convenient (Nav2's goal checker ignores the
heading). A place's saved yaw is only used for its arrow in RViz.

    python3 places_node.py --ros-args -p places_file:=/tmp/places.yaml -p use_sim_time:=true
"""

# ok so this is the node that actually makes the robot move.
# you ask it to go somewhere (a saved place, an object that perception found, or just an x,y point),
# it turns that into a goal for Nav2, and then tells the middleman and Pete how it went.

import json  # the middleman sends its mode as a JSON string
import math
import signal  # only used at the very end, to stop cleanly
import os

import rclpy
import yaml  # the saved places are kept in a yaml file
from action_msgs.msg import GoalStatus  # tells us if a Nav2 goal succeeded, failed or got cancelled
from geometry_msgs.msg import PointStamped  # a point + the frame it is written in (map, camera_link...)
from nav2_msgs.action import NavigateToPose  # the Nav2 action "drive to this position"
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point  # moves a point from one frame into another
from tf2_ros import Buffer, TransformException, TransformListener  # TF = the thing that knows where every frame is
from visualization_msgs.msg import Marker, MarkerArray  # the arrows and names we draw in RViz


def normalise(name):
    """'  The Kitchen ' -> 'kitchen', the way the middleman writes place names."""
    # lowercase, and squash the extra spaces ("  Living   Room " -> "living room")
    name = ' '.join(name.lower().split())
    # the middleman removes "the" from what people say, so we do the same or the names wouldn't match
    return name[4:] if name.startswith('the ') else name


def yaw_of(q):
    """The heading (radians) of a quaternion, for a robot on flat ground."""
    # ROS stores rotations as quaternions (x, y, z, w), which are hard to read. our robot drives on the floor so all we
    # care about is the yaw = which way it is facing. this is just the standard formula to get it out
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def set_yaw(q, yaw):
    """Turn the quaternion q to the heading yaw (radians)."""
    # the opposite of yaw_of: a turn of `yaw` around the vertical axis, written as a quaternion
    q.x, q.y, q.z, q.w = 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


def stop_short_of(robot, target, distance):
    """Where to stop, (x, y, yaw), to end up `distance` metres before target, coming from robot (x, y, yaw). The yaw
    points at the target, but arrival doesn't check it. If the robot is already that close, it stays where it is."""
    # this is for objects: we don't want to drive INTO the chair, we want to stop a bit before it
    # the vector from the robot to the target, and how far away it is
    dx, dy = target[0] - robot[0], target[1] - robot[1]
    gap = math.hypot(dx, dy)
    # the target is basically where we already are: just stay
    if gap < 1e-3:
        return robot
    # already closer than `distance`: stay here
    if gap <= distance:
        return robot[0], robot[1], math.atan2(dy, dx)
    # otherwise drive along the straight line to the target, but stop `distance` before it.
    # k is the fraction of the line we drive (e.g. 3 m away, stopping 0.8 m short: k = 2.2 / 3)
    k = (gap - distance) / gap
    return robot[0] + dx * k, robot[1] + dy * k, math.atan2(dy, dx)


class PlacesNode(Node):
    def __init__(self):
        super().__init__('places')  # 'places' is the node's name in ROS (what ros2 node list shows)

        # parameters: they can be changed when starting the node (-p name:=value), the launch file does that
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value  # the robot's own frame
        # how far from an object's centre the robot stops: clear of the object and of its inflation
        self.object_standoff = self.declare_parameter('object_standoff', 0.8).value
        # where the saved places are kept on disk. expanduser turns the "~" into the home folder
        self.places_file = os.path.expanduser(self.declare_parameter('places_file', '~/.ros/navigoat_places.yaml').value)
        self.places = self.load_places()  # name -> {'x', 'y', 'yaw'} on the map

        # TF: the buffer remembers the transforms between all the frames (map -> odom -> base_link -> camera_link...)
        # and the listener keeps filling it from the /tf topic. we use it to know where the robot is on the map
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Nav2 is driven with an "action": we send it a goal, it drives there, and later it tells us how it went
        self.navigator = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # publishers = the topics we send things on
        self.done_pub = self.create_publisher(String, '/middleman/done', 10)  # "we arrived", for the middleman
        self.response_pub = self.create_publisher(String, '/pete/response', 10)  # Pete says this out loud
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)  # what we're doing, for debugging
        # transient local: RViz still gets the markers if it starts after this node
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.markers_pub = self.create_publisher(MarkerArray, '/navigation/places_markers', latched)
        # the saved names as a JSON list, for the middleman: it understands them next to its own places. Also latched,
        # so a middleman that starts later still gets them
        self.names_pub = self.create_publisher(String, '/navigation/places', latched)

        # subscribers = the topics we listen to. when a message comes in, the function at the end gets called with it
        self.create_subscription(String, '/navigation/save_place', self.on_save, 10)
        self.create_subscription(String, '/navigation/forget_place', self.on_forget, 10)
        self.create_subscription(String, '/navigation/goto', self.on_goto, 10)
        self.create_subscription(PointStamped, '/navigation/goto_point', self.on_goto_point, 10)
        self.create_subscription(PointStamped, '/object_mapping', self.on_object, 10)
        self.create_subscription(String, '/navigation/cancel', self.on_cancel, 10)
        self.create_subscription(String, '/middleman/mode', self.on_middleman_mode, 10)

        # what we're driving to right now. when we send a new goal or cancel, the old one is forgotten, and anything
        # Nav2 says about an older goal is ignored (otherwise an old goal ending could say "arrived" for the wrong place)
        self.request = None  # the goal sent, until Nav2 accepts it
        self.goal = None  # the goal Nav2 accepted
        self.result = None  # its result, still to come
        self.label = None  # what the robot drives to, for the status topic: 'kitchen', 'chair', 'point 1.00,2.00'
        self.spoken = None  # the same, for Pete to say: 'the kitchen', 'that point'
        self.waiting_for = None  # in object mode: the object perception is looking for

        # draw the saved places in RViz straight away, and print what we loaded
        self.publish_markers()
        saved = ', '.join(sorted(self.places)) or 'none yet'
        self.get_logger().info(f'Places ready, {len(self.places)} saved in {self.places_file}: {saved}')

    # --- the places file ---

    def load_places(self):
        # the very first time there is no file yet, so no places
        if not os.path.exists(self.places_file):
            return {}
        with open(self.places_file) as f:
            # the file is {'places': {...}}. the "or {}" bits are in case the file is empty
            places = (yaml.safe_load(f) or {}).get('places') or {}
        # clean up the names, in case someone edited the file by hand
        return {normalise(name): place for name, place in places.items()}

    def save_places(self):
        # make the folder if it doesn't exist yet, then write all the places back into the file
        os.makedirs(os.path.dirname(self.places_file) or '.', exist_ok=True)
        with open(self.places_file, 'w') as f:
            # default_flow_style=None writes each place on one line: kitchen: {x: 1.2, y: 3.4, yaw: 0.5}
            yaml.safe_dump({'places': self.places}, f, default_flow_style=None)

    # --- small helpers ---

    def status(self, text):
        # print it in the terminal AND publish it, so `ros2 topic echo /navigation/status` shows it too
        self.get_logger().info(text)
        self.status_pub.publish(String(data=text))

    def say(self, text):
        # the voice node speaks out loud whatever comes on /pete/response
        self.response_pub.publish(String(data=text))

    def transform_to_map(self, frame, stamp):
        """TF's transform from frame to the map at that time, or the latest one if TF doesn't have that time."""
        # if TF doesn't have that exact time (too old, or a tiny bit in the future), we just use the latest one
        if not self.tf_buffer.can_transform(self.map_frame, frame, stamp):
            stamp = Time()  # the latest
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, frame, stamp)
        except TransformException as e:
            # this happens when the map frame doesn't exist yet, e.g. RTAB-Map hasn't found where the robot is
            self.status(f'error:no transform from {frame} to {self.map_frame}. Is localisation running? ({e})')
            return None

    def robot_pose(self):
        """(x, y, yaw) of the robot on the map, or None."""
        # the transform from the robot's frame to the map IS where the robot is on the map. Time() = the latest
        t = self.transform_to_map(self.base_frame, Time())
        if t is None:
            return None
        return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)

    def point_on_map(self, msg):
        """(x, y) on the map of a PointStamped, using where its frame was at the message's time."""
        # we use the time in the message: e.g. the camera saw the object then, and the robot may have moved since
        t = self.transform_to_map(msg.header.frame_id or self.map_frame, Time.from_msg(msg.header.stamp))
        if t is None:
            return None
        p = do_transform_point(msg, t).point  # the same point, now in map coordinates
        return p.x, p.y

    def publish_markers(self):
        """An orange arrow and the name for every saved place, for RViz. Also sends the names to the middleman."""
        self.names_pub.publish(String(data=json.dumps(sorted(self.places))))
        # start with a DELETEALL so that places we forgot also disappear from RViz
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for i, (name, place) in enumerate(sorted(self.places.items())):
            # every marker needs its own id: the arrow gets an even number and its text the odd number after it
            arrow = Marker(type=Marker.ARROW, ns='places', id=2 * i)
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.5, 0.08, 0.08  # length, width, height of the arrow
            text = Marker(type=Marker.TEXT_VIEW_FACING, ns='places', id=2 * i + 1, text=name)
            text.scale.z = 0.25  # height of the letters
            for m in (arrow, text):
                # both go at the place, pointing the way the robot faced when the place was saved, in orange
                m.header.frame_id = self.map_frame
                m.pose.position.x, m.pose.position.y = place['x'], place['y']
                set_yaw(m.pose.orientation, place['yaw'])
                m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.6, 0.0, 1.0
            text.pose.position.z = 0.4  # the name floats a bit above the arrow
            markers.markers += [arrow, text]
        self.markers_pub.publish(markers)

    # --- driving with Nav2 ---

    def drive_to(self, label, spoken, x, y, yaw):
        """Send Nav2 to (x, y) on the map. A new goal replaces the one being driven to. Nav2 needs a heading in the goal,
        so yaw is the direction that's convenient to arrive in, but arrival only checks the position (the goal
        checker's yaw_goal_tolerance covers every direction)."""
        # if Nav2 isn't running there's nothing we can do (we wait 2 s max for it)
        if not self.navigator.wait_for_server(timeout_sec=2.0):
            self.status('error:Nav2 is not running (no navigate_to_pose action server)')
            self.say("I can't drive right now.")
            return
        # the goal = a position on the map, plus the heading Nav2 needs (not checked on arrival)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        set_yaw(goal.pose.pose.orientation, yaw)
        self.label, self.spoken = label, spoken
        self.goal = self.result = None  # from now on, answers about the previous goal are ignored
        self.status(f'going:{label}')
        # send it. "async" means we don't wait here: when Nav2 answers, on_accepted gets called
        self.request = self.navigator.send_goal_async(goal)
        self.request.add_done_callback(self.on_accepted)

    def on_accepted(self, request):
        goal = request.result()  # Nav2's answer: did it take the goal or not
        if request is not self.request:  # cancelled or replaced while Nav2 was accepting it
            # we don't want this goal anymore, so if Nav2 took it, cancel it straight away
            if goal.accepted:
                goal.cancel_goal_async()
            return
        if not goal.accepted:
            # Nav2 said no (e.g. the goal is inside a wall)
            self.finished(GoalStatus.STATUS_ABORTED)
            return
        # Nav2 is driving now: keep the goal (so we can cancel it) and ask to be told when it ends (on_result)
        self.goal = goal
        self.result = goal.get_result_async()
        self.result.add_done_callback(self.on_result)

    def on_result(self, result):
        # the drive has ended. we only report it if it's the goal we care about now, not an older one
        if result is self.result:  # not about an older goal
            self.finished(result.result().status)

    def finished(self, code):
        # remember the names for the messages, then clear everything: we're not driving anymore
        label, spoken = self.label, self.spoken
        self.request = self.goal = self.result = self.label = None
        if code == GoalStatus.STATUS_SUCCEEDED:
            # we made it: tell the status topic and the middleman (the middleman then says "Arrived.")
            self.status(f'arrived:{label}')
            self.done_pub.publish(String(data=label))
        elif code == GoalStatus.STATUS_CANCELED:
            # someone cancelled it (a stop, follow mode...), nothing to say out loud
            self.status(f'cancelled:{label}')
        else:
            # Nav2 gave up (no path, robot stuck...), so we tell the user
            self.status(f'failed:{label}')
            self.say(f"I couldn't get to {spoken}.")

    def cancel(self):
        """Stop driving, and stop waiting for an object."""
        self.waiting_for = None
        if self.label is None:
            return  # we weren't driving anyway
        if self.goal is not None:
            self.goal.cancel_goal_async()  # tell Nav2 to stop, the robot stops moving
        # (if Nav2 hasn't accepted the goal yet, self.goal is still None: on_accepted will cancel it when it does)
        self.status(f'cancelled:{self.label}')
        self.request = self.goal = self.result = self.label = None

    def go_to_place(self, name):
        place = self.places.get(name)  # None if we never saved that name
        if place is None:
            self.status(f'unknown:{name}')
            self.say(f"I don't know where the {name} is yet.")
            return
        # drive to where the robot was when the place was saved. the way it faced then doesn't matter: the goal's
        # heading is the direction from the robot to the place, the way it naturally arrives. (the sim's planner plans
        # to the exact heading, so the saved one could make it loop round at the end.) arrival only checks the position
        robot = self.robot_pose()
        yaw = math.atan2(place['y'] - robot[1], place['x'] - robot[0]) if robot else place['yaw']
        self.drive_to(name, f'the {name}', place['x'], place['y'], yaw)

    # --- topics ---
    # these get called when a message arrives on the topics we subscribed to in __init__

    def on_save(self, msg):
        # remember where the robot is right now, under the name in the message
        name = normalise(msg.data)
        pose = self.robot_pose()
        if not name:
            return
        if pose is None:  # we don't know where the robot is (robot_pose has already said why on the status topic)
            self.status(f'error:could not save {name}')
            self.say(f"I couldn't save {name}. I don't know where I am.")
            return
        # rounded to the millimetre so the yaml file stays readable
        self.places[name] = {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 3)}
        self.save_places()  # write the file straight away, so it's still there after a restart
        self.publish_markers()
        self.status(f'saved:{name}')
        self.say(f'Saved {name}.')  # the middleman has only said "Remembering this place": this is how it went

    def on_forget(self, msg):
        name = normalise(msg.data)
        # pop removes the place from the dict, and gives None if it wasn't there
        if self.places.pop(name, None) is None:
            self.status(f'unknown:{name}')
            return
        self.save_places()
        self.publish_markers()
        self.status(f'forgot:{name}')

    def on_goto(self, msg):
        # drive to a saved place by name (handy for testing without the middleman)
        self.go_to_place(normalise(msg.data))

    def on_cancel(self, msg):
        # any message on /navigation/cancel stops the robot, whatever it says
        self.cancel()

    def on_goto_point(self, msg):
        # drive to any point. the point can be in any frame, so first we put it on the map
        point = self.point_on_map(msg)
        robot = self.robot_pose()
        if point is None or robot is None:
            return
        yaw = math.atan2(point[1] - robot[1], point[0] - robot[0])  # the way it drives there (not checked on arrival)
        self.drive_to(f'point {point[0]:.2f},{point[1]:.2f}', 'that point', point[0], point[1], yaw)

    def on_object(self, msg):
        """Perception found the object: in object mode, drive to object_standoff metres short of it."""
        name = self.waiting_for
        if name is None:
            # perception also gets tested on its own, and we only want to drive in object mode
            self.get_logger().info('/object_mapping received outside object mode, not driving.')
            return
        point = self.point_on_map(msg)  # perception gives it relative to the camera, we need it on the map
        robot = self.robot_pose()
        if point is None or robot is None:
            return
        self.waiting_for = None  # got it, stop waiting
        x, y, yaw = stop_short_of(robot, point, self.object_standoff)  # stop before it
        self.get_logger().info(f'The {name} is at ({point[0]:.2f}, {point[1]:.2f}), stopping at ({x:.2f}, {y:.2f}).')
        self.drive_to(name, f'the {name}', x, y, yaw)

    def on_middleman_mode(self, msg):
        """The middleman's mode, JSON {"mode", "target", "stop"}. "location" drives to a place, "object" waits for
        perception to find the object. A stop, or any other mode (follow, manual, help...), ends the drive."""
        # the middleman sends JSON inside a String, so turn it into a python dict first
        try:
            data = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning(f'Not JSON on /middleman/mode: {msg.data!r}')
            return
        mode, target = data.get('mode'), normalise(data.get('target') or '')
        # anything that isn't "go to a place" or "go to an object" means we should stop driving
        if data.get('stop') or not target or mode not in ('location', 'object'):
            self.cancel()
        elif mode == 'location':
            # go to a saved place
            self.waiting_for = None
            self.go_to_place(target)
        else:
            # object mode: stop and wait for perception to find it, then on_object drives there
            self.cancel()
            self.waiting_for = target
            self.status(f'waiting:{target}')


def main():
    rclpy.init()  # start ROS in this program
    node = PlacesNode()
    try:
        rclpy.spin(node)  # keeps the node running, and calls our functions when messages come in
    except BaseException:  # Ctrl+C, the launch file stopping us, or a real error
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch sends Ctrl+C a second time: ignore it
        if rclpy.ok():  # ROS is still running, so this is a real error: show it
            raise


# this runs main() when the file is started with: python3 places_node.py
if __name__ == '__main__':
    main()
