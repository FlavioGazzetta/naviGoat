"""Keeps the people around the robot on the map

People come from three places, and are all published together on /people, 10 times a second:
  - Enid's tracker: positions relative to the camera, put on the map with where the camera was when the image was
    taken. A person's speed comes from how they moved on the map over the last velocity_window seconds, so the robot
    turning doesn't make standing people look like they walk. A walker is moved forward by how old the image is, so
    the costmaps get where they are now, not where they were when the camera saw them. Someone standing is remembered for remember_for seconds after the camera
    loses them (the robot may only have turned away), someone walking for track_timeout seconds.
  - RViz's "Publish Point" tool: a click adds a standing person, a click on one removes them.
  - fake_people, a YAML file of made-up people for testing (see ../config/fake_people.yaml): standing people, and
    people walking back and forth between two points.

    python3 people_node.py --ros-args -p use_sim_time:=true -p fake_people:=../config/fake_people.yaml
"""

# this node collects all the people we know about and puts them on the map, so the social layer (in the costmaps)
# can keep the robot away from them. it doesn't detect anyone itself: it just takes coordinates from the tracker,
# from RViz clicks and from the fake people file.

import collections  # for deque, a list that is quick to remove things from the front
import json  # Enid's tracker sends JSON inside a String
import math
import signal  # only used at the very end, to stop cleanly
import threading  # to listen to the tracker on another ROS domain, see relay_tracks_from()

import numpy as np  # for the line fit that gives the walking speed
import rclpy
import yaml  # the fake people file
from geometry_msgs.msg import Point, PointStamped
from navigoat_msgs.msg import People, Person  # our own messages, see navigoat_msgs/msg/
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_point  # moves a point from one frame into another
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray  # the cylinders and arrows we draw in RViz

# a person counts as walking if they move faster than this, otherwise they're standing
WALKING = 0.25  # m/s: anyone slower is standing (the same as the social layer's min_speed)


class Track:
    """One person from the tracker: where they were on the map over the last `window` seconds."""

    def __init__(self, window=1.0, min_span=0.5):
        self.window = window  # s of positions the speed is fitted on
        self.min_span = min_span  # s of positions needed before the speed is trusted
        # we use the deque as a sliding window: the last `window` seconds of positions of this person
        self.history = collections.deque()  # (stamp, x, y), stamp = when the image was taken, in seconds
        self.last_seen = 0.0  # when we last got them, in seconds

    def add(self, stamp, x, y, now):
        self.history.append((stamp, x, y))
        # throw away the positions older than the window
        while stamp - self.history[0][0] > self.window:
            self.history.popleft()
        self.last_seen = now

    def last_stamp(self):
        # when the image they were last seen in was taken
        return self.history[-1][0]

    def position(self):
        # where they were the last time we saw them
        return self.history[-1][1], self.history[-1][2]

    def velocity(self):
        """(vx, vy) in m/s: the slope of the best-fitting line through the window. 0 until there is min_span of it."""
        # split the history into 3 arrays: the times, the x's and the y's
        t, x, y = (np.array(column) for column in zip(*self.history))
        # with too little data the speed would just be noise, so we say they're standing
        if len(t) < 3 or t[-1] - t[0] < self.min_span:
            return 0.0, 0.0
        t = t - t[0]  # start the times at 0 (the real timestamps are huge numbers, which the line fit doesn't like)
        # fit a straight line through x over time: its slope is the speed along x. same thing for y.
        # a line through all the points is less noisy than just (last position - first position) / time
        return float(np.polyfit(t, x, 1)[0]), float(np.polyfit(t, y, 1)[0])


def load_fake_people(path):
    """The standing [(x, y)] and walking [(start, end, speed)] people in a YAML file. Nothing if path is '' or 'none'."""
    # no file given: no fake people
    if path in ('', 'none'):
        return [], []
    with open(path) as f:
        people = (yaml.safe_load(f) or {}).get('people') or []
    # someone with x and y stands still, someone with from and to walks between the two points
    standing = [(p['x'], p['y']) for p in people if 'x' in p]
    walking = [(p['from'], p['to'], p.get('speed', 1.0)) for p in people if 'from' in p]
    return standing, walking


def walker_at(start, end, speed, t):
    """Where someone walking back and forth between start and end is after t seconds, and their velocity."""
    # the line they walk on, and its length
    dx, dy = end[0] - start[0], end[1] - start[1]
    length = math.hypot(dx, dy)
    if length == 0 or speed <= 0:
        return start, (0.0, 0.0)  # not really walking
    # one trip there and back is 2 * length, so % tells us how far into the current trip they are
    s = (t * speed) % (2 * length)  # distance walked since the last time at start
    direction = 1.0
    if s > length:  # on the way back
        s = 2 * length - s  # how far from start they are, coming back
        direction = -1.0
    ux, uy = dx / length, dy / length  # the direction of the line, as a vector of length 1
    return (start[0] + ux * s, start[1] + uy * s), (direction * speed * ux, direction * speed * uy)


class PeopleNode(Node):
    def __init__(self):
        super().__init__('people')
        # parameters (see the README)
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.camera_frame = self.declare_parameter('camera_frame', 'camera_link').value  # the tracker's x forward, y left
        self.track_timeout = self.declare_parameter('track_timeout', 0.5).value  # s: then a walker we lost is dropped
        self.remember_for = self.declare_parameter('remember_for', 3.0).value  # s: how long standing people are kept
        # the speed is a line fitted through the last velocity_window seconds, once there are min_velocity_span of them.
        # shorter = a walker's speed (and the social layer's prediction) shows up sooner, but is noisier. was 1.0 / 0.5
        self.velocity_window = self.declare_parameter('velocity_window', 0.8).value  # s
        self.min_velocity_span = self.declare_parameter('min_velocity_span', 0.4).value  # s
        # a walker is moved forward by how old the image is (YOLO + tracker latency), at most this much
        self.max_extrapolation = self.declare_parameter('max_extrapolation', 0.5).value  # s
        self.standing, self.walking = load_fake_people(self.declare_parameter('fake_people', 'none').value)
        # -1: the tracker is on our ROS domain. otherwise the domain it's on (see relay_tracks_from)
        tracker_domain = self.declare_parameter('tracker_domain', -1).value
        self.relay_context = None  # set by relay_tracks_from() when the tracker is on another domain

        self.tracks = {}  # tracker id -> Track
        self.clicked = {}  # id (negative) -> (x, y), the people added in RViz
        self.start = self.seconds()  # the fake walkers start walking when the node starts

        # TF, to move the tracker's points from the camera's frame to the map (see places_node for more)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # what we send: the people for the social layer, and the markers for RViz
        self.people_pub = self.create_publisher(People, '/people', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/navigation/people_markers', 10)
        # what we listen to: Enid's tracker, and clicks in RViz
        self.create_subscription(String, '/social_perception/tracks', self.on_tracks, 10)
        self.create_subscription(PointStamped, '/clicked_point', self.on_click, 10)
        if tracker_domain >= 0:
            self.relay_tracks_from(tracker_domain)
        # publish everyone 10 times a second (every 0.1 s)
        self.create_timer(0.1, self.publish)
        self.get_logger().info(f'People ready: {len(self.standing)} standing and {len(self.walking)} walking fake '
                               'people. Click in RViz with "Publish Point" to add or remove one.')

    def relay_tracks_from(self, domain):
        """Listen to Enid's tracker on another ROS domain, and pass its messages on to /social_perception/tracks here.

        For real people on the simulated map (navigation_sim.launch.py real_people:=true): the simulation then runs on its
        own domain, so the real camera's /camera/... images, which are on the tracker's domain, don't mix with Gazebo's."""
        if domain == self.context.get_domain_id():
            return  # it's our own domain: the normal subscription already gets the tracks
        # a second, separate ROS "connection" on the tracker's domain, with one node that only listens to the tracks
        self.relay_context = rclpy.Context()
        rclpy.init(context=self.relay_context, domain_id=domain)
        listener = rclpy.create_node('people_tracks_listener', context=self.relay_context)
        # each message it gets is published again here, where our own on_tracks subscription picks it up
        relay = self.create_publisher(String, '/social_perception/tracks', 10)

        def forward(msg):
            try:
                relay.publish(msg)
            except Exception:
                pass  # we're stopping (Ctrl+C) and our side is already closed: nothing to forward to

        listener.create_subscription(String, '/social_perception/tracks', forward, 10)
        # that node needs its own spin, next to rclpy.spin(node) in main(): a background thread does it
        self.relay_executor = SingleThreadedExecutor(context=self.relay_context)
        self.relay_executor.add_node(listener)

        def spin():
            try:
                self.relay_executor.spin()
            except ExternalShutdownException:
                pass  # Ctrl+C or the launch file stopping us closed this connection too: a normal stop

        self.relay_thread = threading.Thread(target=spin)
        self.relay_thread.start()
        self.get_logger().info(f'Listening to the people tracker on ROS domain {domain}.')

    def stop_relay(self):
        """Stop the thread of relay_tracks_from(), if there is one. Without this, the program crashes when it stops."""
        if self.relay_context is None:
            return
        self.relay_executor.shutdown(timeout_sec=1.0)  # makes its spin() return
        self.relay_thread.join(timeout=1.0)  # wait for the thread to finish
        self.relay_context.try_shutdown()  # and close the connection to the tracker's domain

    def seconds(self):
        # the time now, in seconds (in Gazebo this is the simulation's time)
        return self.get_clock().now().nanoseconds / 1e9

    def transform_to_map(self, frame, stamp):
        """TF's transform from frame to the map at that time, or the latest one if TF doesn't have that time."""
        # if TF doesn't have that exact time, we just use the latest one
        if not self.tf_buffer.can_transform(self.map_frame, frame, stamp):
            stamp = Time()  # the latest
        try:
            return self.tf_buffer.lookup_transform(self.map_frame, frame, stamp)
        except TransformException as e:
            # throttle_duration_sec: print this at most once every 5 s, so it doesn't flood the terminal
            self.get_logger().warning(f'Cannot place people from {frame} on the map: {e}', throttle_duration_sec=5.0)
            return None

    def on_tracks(self, msg):
        """Enid's tracker: {"stamp_sec": ..., "tracks": [{"id", "x", "y", "confirmed"}]}, x forward and y left of the
        camera, in metres."""
        # turn the JSON text into a dict. if it's broken, we just ignore that message
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        now = self.seconds()
        stamp = data.get('stamp_sec') or now  # when the image was taken (or now, if the tracker didn't say)
        # where the camera was when it took the image. that matters when the robot is turning: by the time the
        # tracker is done the camera already looks somewhere else
        transform = self.transform_to_map(self.camera_frame, Time(seconds=stamp))
        if transform is None:
            return
        for track in data.get('tracks', []):
            # the tracker also has people it isn't sure about yet: we skip those
            if not track.get('confirmed', True):
                continue
            # and people it didn't see in this image, only predicted. its prediction assumes the camera stands still,
            # so as soon as the robot moves or turns it drifts away from the real person. we remember people ourselves
            # (remember_for / track_timeout), so only real detections go in
            if not track.get('observed', True):
                continue
            # the person relative to the camera -> the same point on the map
            point = PointStamped()
            point.point.x, point.point.y = float(track['x']), float(track['y'])
            p = do_transform_point(point, transform).point
            # add it to that person's history, with a new Track the first time we see that id
            if track['id'] not in self.tracks:
                self.tracks[track['id']] = Track(self.velocity_window, self.min_velocity_span)
            self.tracks[track['id']].add(stamp, p.x, p.y, now)

    def on_click(self, msg):
        """RViz's Publish Point: add a standing person there, or remove the clicked person within 0.5 m."""
        # RViz clicks are usually already in the map frame, but we transform anyway in case they aren't
        transform = self.transform_to_map(msg.header.frame_id or self.map_frame, Time())
        if transform is None:
            return
        p = do_transform_point(msg, transform).point
        # is there already a clicked person within 0.5 m of the click?
        near = [i for i, (x, y) in self.clicked.items() if math.hypot(x - p.x, y - p.y) < 0.5]
        if near:
            # yes: clicking on someone removes them
            del self.clicked[near[0]]
            self.get_logger().info(f'Removed the person at ({p.x:.1f}, {p.y:.1f}).')
        else:
            # no: add a standing person there. clicked people get the ids -1, -2, -3... so they can never be the
            # same as the tracker's ids (which start at 0)
            self.clicked[min(self.clicked, default=0) - 1] = (p.x, p.y)
            self.get_logger().info(f'Added a person at ({p.x:.1f}, {p.y:.1f}).')

    def everyone(self):
        """[(id, x, y, vx, vy)]: the tracked people (forgetting those not seen for too long), then the fake ones.
        Fake people have negative ids."""
        now = self.seconds()
        people = []
        # the people from the tracker. list() makes a copy, because we delete from the dict inside the loop
        for i, track in list(self.tracks.items()):
            vx, vy = track.velocity()
            walking = math.hypot(vx, vy) >= WALKING
            age = now - track.last_seen  # how long since we last saw them
            # walkers are forgotten quickly (they walk away), standing people slowly (we probably just looked away)
            if age > (self.track_timeout if walking else self.remember_for):
                del self.tracks[i]
            elif age > self.track_timeout:
                people.append((i, *track.position(), 0.0, 0.0))  # not seen lately: kept where they stood
            else:
                # seen just now: where they are and how they walk. a walker is moved forward by how old the image is
                # (YOLO and the tracker take a few tenths of a second), so the costmaps get where they are now.
                # clamped at 0: with real people on the simulation the camera's clock is not the simulation's
                x, y = track.position()
                if walking:
                    dt = min(max(now - track.last_stamp(), 0.0), self.max_extrapolation)
                    x, y = x + vx * dt, y + vy * dt
                people.append((i, x, y, vx, vy))
        # the fake people from the yaml file, with the ids -1000, -1001...
        fake_id = -1000
        for x, y in self.standing:
            people.append((fake_id, x, y, 0.0, 0.0))
            fake_id -= 1
        for start, end, speed in self.walking:
            # where each walker is now, from how long the node has been running
            (x, y), (vx, vy) = walker_at(start, end, speed, now - self.start)
            people.append((fake_id, x, y, vx, vy))
            fake_id -= 1
        # and the people clicked in RViz, standing still
        people += [(i, x, y, 0.0, 0.0) for i, (x, y) in self.clicked.items()]
        return people

    def publish(self):
        # the message for the social layer: everyone, in the map frame, with the time now
        msg = People()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        # For RViz: a cylinder per person, blue if tracked and purple if fake, and an arrow to where a walker will be
        # in 1 s.
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])  # first clear the old markers
        for k, (i, x, y, vx, vy) in enumerate(self.everyone()):
            # the person, for the message
            person = Person(id=i)
            person.position.x, person.position.y = float(x), float(y)
            person.velocity.x, person.velocity.y = float(vx), float(vy)
            msg.people.append(person)

            # a cylinder about the size of a person (0.5 m wide and 1.7 m tall, so its middle is 0.85 m up)
            body = Marker(type=Marker.CYLINDER, ns='people', id=2 * k)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = float(x), float(y), 0.85
            body.scale.x, body.scale.y, body.scale.z = 0.5, 0.5, 1.7
            body.color.r, body.color.g, body.color.b, body.color.a = (0.7, 0.3, 0.9, 0.8) if i < 0 else (0.2, 0.5, 1.0, 0.8)
            markers.markers.append(body)
            # walkers also get an arrow from them to where they'll be in 1 s (position + velocity * 1 s)
            if math.hypot(vx, vy) >= WALKING:
                arrow = Marker(type=Marker.ARROW, ns='people', id=2 * k + 1, color=body.color)
                arrow.points = [Point(x=float(x), y=float(y), z=1.0), Point(x=float(x + vx), y=float(y + vy), z=1.0)]
                arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.06, 0.15, 0.2
                markers.markers.append(arrow)
        # every marker is in the map frame, and needs a valid rotation (w = 1 means "not rotated")
        for m in markers.markers:
            m.header.frame_id = self.map_frame
            m.pose.orientation.w = 1.0
        self.people_pub.publish(msg)
        self.markers_pub.publish(markers)


def main():
    rclpy.init()  # start ROS in this program
    node = PeopleNode()
    try:
        rclpy.spin(node)  # keeps the node running, and calls our functions when messages and the timer come
    except BaseException:  # Ctrl+C, the launch file stopping us, or a real error
        signal.signal(signal.SIGINT, signal.SIG_IGN)  # ros2 launch sends Ctrl+C a second time: ignore it
        if rclpy.ok():  # ROS is still running, so this is a real error: show it
            raise
    finally:
        node.stop_relay()  # however we stop, stop the tracker listener too (see relay_tracks_from)


# this runs main() when the file is started with: python3 people_node.py
if __name__ == '__main__':
    main()
