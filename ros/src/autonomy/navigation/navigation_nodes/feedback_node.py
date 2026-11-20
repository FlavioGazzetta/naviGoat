"""Tells the user what is around and ahead, through the handle or the wristband: how close people are, and the turns
coming up. Started by launch/navigation.launch.py. Its topics are listed in autonomy/navigation/README.md.

Both outputs are std_msgs/UInt8MultiArray [side, magnitude], the format of /handle/vibration and /wristband/vibration:
side 0 is left and 1 right, magnitude goes from 0 to 255. They are published 10 times a second.

    python3 feedback_node.py --ros-args -p use_sim_time:=true
"""

# this node is for the person being guided: it works out two things 10 times a second and sends them in the same
# format as the handle and wristband vibrations:
#   - how close the nearest person is (stronger = closer)
#   - if a turn is coming on the path Nav2 planned (stronger = sharper turn)

import math
import signal  # only used at the very end, to stop cleanly

import rclpy
from nav_msgs.msg import Path  # Nav2's planned path, a list of poses
from navigoat_msgs.msg import People
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import UInt8MultiArray  # a list of numbers from 0 to 255, here [side, magnitude]
from tf2_ros import Buffer, TransformException, TransformListener

# the side numbers the handle and the wristband use
LEFT, RIGHT = 0, 1


def wrap(angle):
    """The same angle, between -pi and pi."""
    # e.g. 270 degrees is the same as -90 degrees. atan2(sin, cos) gives it back between -180 and 180
    return math.atan2(math.sin(angle), math.cos(angle))


def proximity(people, warn_distance, min_distance):
    """(side, magnitude) for the closest of people [(forward, left)], given relative to the robot in metres:
    0 at warn_distance or further, rising to 255 at min_distance or closer."""
    if not people:
        return LEFT, 0  # nobody around: no vibration
    # the closest person
    forward, left = min(people, key=lambda p: math.hypot(*p))
    # goes from 0 at warn_distance to 1 at min_distance, in a straight line in between
    level = (warn_distance - math.hypot(forward, left)) / (warn_distance - min_distance)
    # left >= 0 means they're on our left. the level is clamped between 0 and 1, then turned into 0..255
    return (LEFT if left >= 0 else RIGHT), round(255 * min(1.0, max(0.0, level)))


def upcoming_turn(path, robot, warn_distance, min_turn):
    """The angle (radians, positive to the left) of the first turn on path [(x, y)] that starts within warn_distance of
    the robot (x, y, yaw), or None if the path goes straight. The path's heading is measured over 0.4 m, so the small
    zigzags of the planner's grid don't count, and against the way the robot faces, so turning on the spot counts."""
    if len(path) < 2:
        return None  # a path of one point doesn't go anywhere
    # Nav2's path starts where the robot was when it planned, so first we find where we are on it now
    start = min(range(len(path)), key=lambda k: math.dist(path[k], robot[:2]))  # the path point nearest the robot
    points = path[start:]  # only the part ahead of us
    along = [0.0]  # how far along the path each point is
    for a, b in zip(points, points[1:]):
        along.append(along[-1] + math.dist(a, b))  # add up the length of each little piece of the path
    end = along[-1] - 0.4  # the last place a heading can be measured

    def point_at(distance):
        """The first point at least `distance` metres along the path."""
        for point, d in zip(points, along):
            if d >= distance:
                return point
        return points[-1]

    def turned(distance):
        """How much the path's heading `distance` metres along differs from the robot's heading."""
        # the path's direction there = the direction from that point to the point 0.4 m further on
        (ax, ay), (bx, by) = point_at(distance), point_at(distance + 0.4)
        return wrap(math.atan2(by - ay, bx - ax) - robot[2])

    # walk along the path in 10 cm steps, up to warn_distance ahead, until its direction is off by min_turn or more
    distance = 0.0
    while distance <= min(warn_distance, end):
        if abs(turned(distance)) >= min_turn:
            # the whole turn: the biggest change of heading over the next 1.5 m
            return max((turned(min(distance + k * 0.1, end)) for k in range(16)), key=abs)
        distance += 0.1
    return None  # no turn coming


class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback')
        # parameters (the README has a table of what they do)
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value  # to tell whether the robot is moving
        self.warn_distance = self.declare_parameter('warn_distance', 2.0).value  # m: closer people are felt (was 1.2)
        self.min_distance = self.declare_parameter('min_distance', 0.5).value  # m: this close, full strength
        # was True (people felt only while driving): now a person near the robot is felt whether it drives or not
        self.only_when_moving = self.declare_parameter('only_when_moving', False).value
        self.turn_warn_distance = self.declare_parameter('turn_warn_distance', 2.0).value  # m: how far ahead turns are felt
        self.min_turn = math.radians(self.declare_parameter('min_turn_deg', 25.0).value)  # smaller bends aren't turns
        self.full_turn = math.radians(self.declare_parameter('full_turn_deg', 90.0).value)  # this sharp: 255

        # TF, to know where the robot is
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # the two outputs
        self.proximity_pub = self.create_publisher(UInt8MultiArray, '/navigation/proximity_feedback', 10)
        self.turn_pub = self.create_publisher(UInt8MultiArray, '/navigation/turn_feedback', 10)
        # the inputs: the people from people_node, and the path Nav2 planned
        self.create_subscription(People, '/people', self.on_people, 10)
        self.create_subscription(Path, '/plan', self.on_plan, 10)
        # work out and send both outputs 10 times a second
        self.create_timer(0.1, self.tick)

        self.people = None  # the latest people message
        self.plan = None  # the latest path from Nav2
        self.plan_time = 0.0  # when we got it
        self.last_position = None  # (time, x, y) of the robot on the map a tick ago
        self.speed = 0.0  # how fast the robot is going, in m/s
        self.side = {}  # publisher -> the side it sent last
        self.get_logger().info('Feedback ready: /navigation/proximity_feedback and /navigation/turn_feedback.')

    def seconds(self):
        # the time now, in seconds (in Gazebo this is the simulation's time)
        return self.get_clock().now().nanoseconds / 1e9

    def on_people(self, msg):
        # just keep the latest one, tick() uses it
        self.people = msg

    def on_plan(self, msg):
        # same for the path, and we note when it came, to know if Nav2 is still driving
        self.plan = msg
        self.plan_time = self.seconds()

    def robot_pose(self, frame):
        """(x, y, yaw) of the robot in frame, or None."""
        try:
            # where the robot is in that frame. Time() = the latest we know
            t = self.tf_buffer.lookup_transform(frame, self.base_frame, Time()).transform
        except TransformException:
            return None  # TF doesn't know yet (e.g. right at startup)
        q = t.rotation
        # the yaw from the quaternion, same formula as yaw_of() in places_node.py
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def tick(self):
        # first, how fast is the robot moving? from how far it went since the last tick
        position = self.robot_pose(self.map_frame)
        now = self.seconds()
        if position is not None:
            if self.last_position is not None and now > self.last_position[0]:
                t, x, y = self.last_position
                # speed = distance / time. we keep 70% of the old value and add 30% of the new one to smooth it out
                self.speed = 0.7 * self.speed + 0.3 * math.hypot(position[0] - x, position[1] - y) / (now - t)
            self.last_position = (now, position[0], position[1])
        # then send both signals
        self.send(self.proximity_pub, *self.people_feedback())
        self.send(self.turn_pub, *self.turn_feedback())

    def people_feedback(self):
        """(side, magnitude) for the closest person."""
        # no people yet, or the robot is standing still and we only want this while driving
        if self.people is None or (self.only_when_moving and self.speed < 0.05):
            return LEFT, 0
        # where the robot is, in the same frame as the people (the map)
        robot = self.robot_pose(self.people.header.frame_id)
        if robot is None:
            return LEFT, 0
        x, y, yaw = robot
        around = []  # each person as (forward, left) of the robot
        for p in self.people.people:
            dx, dy = p.position.x - x, p.position.y - y  # from the robot to the person, on the map
            # turn that by the robot's heading, so it's from the robot's point of view (x forward, y left)
            around.append((math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy))
        return proximity(around, self.warn_distance, self.min_distance)

    def turn_feedback(self):
        """(side, magnitude) for the next turn on Nav2's path."""
        # Nav2 sends a new path about once a second while it drives: an older one means it has finished.
        if self.plan is None or not self.plan.poses or self.seconds() - self.plan_time > 2.0:
            return LEFT, 0
        robot = self.robot_pose(self.plan.header.frame_id)
        if robot is None:
            return LEFT, 0
        path = [(p.pose.position.x, p.pose.position.y) for p in self.plan.poses]  # just the x, y of each point
        angle = upcoming_turn(path, robot, self.turn_warn_distance, self.min_turn)
        if angle is None:
            return LEFT, 0  # no turn coming
        # a positive angle is a left turn. the sharper the turn, the stronger (full_turn or sharper = 255)
        return (LEFT if angle > 0 else RIGHT), round(255 * min(1.0, abs(angle) / self.full_turn))

    def send(self, publisher, side, magnitude):
        """Publish [side, magnitude]. When the vibration moves to the other side, first stop the side that was on, or
        it would keep vibrating."""
        last_side = self.side.get(publisher)
        # the devices keep a side vibrating until they get a 0 for that side, so switch the old side off first
        if last_side is not None and last_side != side:
            publisher.publish(UInt8MultiArray(data=[last_side, 0]))
        publisher.publish(UInt8MultiArray(data=[side, magnitude]))
        self.side[publisher] = side


def main():
    rclpy.init()  # start ROS in this program
    node = FeedbackNode()
    try:
        rclpy.spin(node)  # keeps the node running, and calls our functions when messages and the timer come
    except BaseException:  # Ctrl+C, the launch file stopping us, or a real error
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch sends Ctrl+C a second time: ignore it
        if rclpy.ok():  # ROS is still running, so this is a real error: show it
            raise


# this runs main() when the file is started with: python3 feedback_node.py
if __name__ == '__main__':
    main()
