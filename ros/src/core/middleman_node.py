"""ROS 2 node around the middleman. Run it with launch/middleman.launch.py. The README in this folder has the topics table.

    speech_topic         /speech/text         in   what the user said, from speech to text
    response_topic       /pete/response       out  what Pete says: replies from the middleman, and the scene description
    middleman_mode_topic /middleman/mode      out  JSON {"mode", "target", "intent", "stop", "request_id"}
    following_status_topic /person_following/status in JSON {"request_id", "state", "event", "target_id"}
    heard_topic          /middleman/heard     out  published when a request goes to perception (no spoken reply); play a beep on it
    mode_topic, goal_topic  /mode, /goal      out  what perception listens to (autonomy/perception/README.md): an object request
                                                   is /goal = the object then /mode = perception, a scene request is /mode =
                                                   description, anything else is /mode = the mode's name, or idle
    status_topic         /object_status       in   perception's search result, 'found:<object>' or 'not_found:<object>'; a failed
                                                   search is spoken ("I couldn't find the chair.") and the mode goes back to idle
    description_topic    /scene_description   in   perception's description of the camera view, spoken through the response topic
    done_topic           /middleman/done      in   the robot reached the goal of a location, object or GPS request
    failed_topic         /middleman/failed    in   the robot gave up on a location, object or GPS request: the text is the reason,
                                                   ready to be spoken; it is said and the mode goes back to idle
    save_place_topic     /navigation/save_place  out  the name from "remember this place as X"; navigation saves the robot's position under it
    places_topic         /navigation/places   in   JSON list of the names navigation has saved (latched): understood like the catalog's places

Every topic name above is a ROS parameter (the name in the first column, the default in the second), and so is use_qwen.

Modes other than perception read /middleman/mode, including the following camera-test runner.
Goal modes end when the robot publishes /middleman/done, or /middleman/failed when it gives up. Following and manual
mode have no goal: they end when the user says "stop", "exit manual mode" or "stop following". Following also ends on
correlated failure feedback.

    python3 middleman_node.py --ros-args -p use_qwen:=true
"""

import json
import re
import threading
import uuid

from middleman import DESCRIBE, Middleman, Mode, qwen_interpreter

# The modes that drive to a goal, so they can end in /middleman/done or /middleman/failed
GOAL_MODES = (Mode.LOCATION, Mode.OBJECT, Mode.GPS)
# Said when a failure report comes without a reason
COULD_NOT_GET_THERE = "Sorry, I couldn't get there."

# The topic parameters and their defaults. mode_topic and goal_topic are the names perception listens to.
TOPICS = {
    'speech_topic': '/speech/text', 'response_topic': '/pete/response', 'middleman_mode_topic': '/middleman/mode',
    'heard_topic': '/middleman/heard', 'mode_topic': '/mode', 'goal_topic': '/goal', 'status_topic': '/object_status',
    'description_topic': '/scene_description', 'done_topic': '/middleman/done', 'failed_topic': '/middleman/failed',
    'save_place_topic': '/navigation/save_place', 'places_topic': '/navigation/places',
    'following_status_topic': '/person_following/status'}

CANNOT_ANSWER = "Sorry, I can only describe what is in front of me."


def mode_messages(mode, target=''):
    """The (topic parameter, text) messages that put perception into a mode. mode is a Mode's value, or 'idle'."""
    if mode == Mode.OBJECT.value:
        return [('goal_topic', target), ('mode_topic', 'perception')]  # the goal first, so the search starts with it
    return [('mode_topic', mode)]  # perception stays idle for any mode that is not its own


# "Describe what's in front of me in detail": the long description. Without these words it is the short one.
DETAIL = re.compile(r"\b(in (more |great |full )?detail|detailed|in depth|thoroughly|everything)\b")


def describe_messages(detail=False):
    # The description node ignores a mode it is already in, so leave it first to be able to describe again.
    return [('mode_topic', 'idle'), ('mode_topic', 'description detail' if detail else 'description')]


def mode_payload(mode, target='', intent='', stop=False, request_id=''):
    """Additional request_id is ignored by older consumers of mode messages."""
    return {'mode': 'stop' if stop else mode, 'target': target, 'intent': intent,
            'stop': stop, 'request_id': request_id or uuid.uuid4().hex}


def main():
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('middleman')
    param = lambda name, default: node.declare_parameter(name, default).value
    topic = {name: param(name, default) for name, default in TOPICS.items()}
    use_qwen = param('use_qwen', False)  # let Qwen (through Ollama) match a need like "I'm hungry" to a known place

    response_pub = node.create_publisher(String, topic['response_topic'], 10)
    mode_pub = node.create_publisher(String, topic['middleman_mode_topic'], 10)
    ack_pub = node.create_publisher(String, topic['heard_topic'], 10)
    save_place_pub = node.create_publisher(String, topic['save_place_topic'], 10)
    perception = {name: node.create_publisher(String, topic[name], 10) for name in ('mode_topic', 'goal_topic')}

    interpreter = qwen_interpreter() if use_qwen else None
    if interpreter and not interpreter.warm_up():
        node.get_logger().warning('Cannot reach Ollama, using the rules only.')
        interpreter = None
    middleman = Middleman(interpreter=interpreter, on_ack=lambda: ack_pub.publish(String(data='heard')))
    # Do not hold this lock during language-model interpretation; explicit stops
    # remain responsive. Command/status publication is ordered under this lock.
    command_lock = threading.RLock()

    def send(messages):
        for name, text in messages:
            perception[name].publish(String(data=text))

    def announce(mode, target='', intent='', stop=False, request_id=''):
        """Tell everything else that the mode changed: perception on the mode and goal topics, the rest on the middleman one."""
        send(mode_messages(mode, target))
        mode_pub.publish(String(data=json.dumps(mode_payload(mode, target, intent, stop, request_id))))

    def on_speech(msg):
        decision = middleman.handle(msg.data)
        if decision is None:
            return
        with command_lock:
            if decision.mode is not None and middleman.current is not decision:
                return  # superseded before publication
            if decision.forward:
                if DESCRIBE.search(decision.forward):
                    send(describe_messages(bool(DETAIL.search(decision.forward.lower()))))
                else:
                    response_pub.publish(String(data=CANNOT_ANSWER))
            if decision.save_place:
                save_place_pub.publish(String(data=decision.save_place))
            if decision.stop:
                announce('idle', stop=True, request_id=decision.request_id)
            elif decision.mode is not None:
                announce(decision.mode.value, decision.target, decision.intent,
                         request_id=decision.request_id)
            if decision.reply:
                response_pub.publish(String(data=decision.reply))

    def on_following_status(msg):
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError, RecursionError):
            node.get_logger().warning('Ignored invalid following-status JSON')
            return
        with command_lock:
            decision = middleman.following_status(payload)
            if decision is None or middleman.current is not decision:
                return
            announce('idle', request_id=decision.request_id)
            response_pub.publish(String(data=decision.reply))

    def on_done(msg):
        """Whoever drives a location / object / GPS goal says it has arrived. Following and manual have no goal."""
        middleman.finish()
        announce('idle')
        response_pub.publish(String(data='Arrived.'))

    def on_failed(msg):
        """Whoever drives a location / object / GPS goal gave up. The message is the reason, ready to be spoken.
        fail() decides what comes next: a stored place can stand in for an object, otherwise the mode goes back to idle."""
        if middleman.mode not in GOAL_MODES:  # a late report, after a stop or another request
            node.get_logger().info(f'Failure report while not driving to a goal, ignored: {msg.data!r}')
            return
        decision = middleman.fail()
        reason = msg.data.strip() or COULD_NOT_GET_THERE
        if decision is not None:  # a stored place stands in for the object
            announce(decision.mode.value, decision.target, decision.intent)
            response_pub.publish(String(data=f'{reason} {decision.reply}'))
        else:
            announce('idle')
            response_pub.publish(String(data=reason))

    def on_object_status(msg):
        """Perception's report at the end of a search: 'found:chair' or 'not_found:chair'. Only a failed search is spoken."""
        status, _, goal = msg.data.partition(':')
        if status != 'not_found':
            return
        decision = middleman.not_found(goal.strip().lower())
        if decision is None:
            return
        if decision.mode is not None:  # a stored place stands in for the object
            announce(decision.mode.value, decision.target, decision.intent)
        else:
            announce('idle')
        response_pub.publish(String(data=decision.reply))

    def on_places(msg):
        """navigation's saved places, a JSON list of names: understood next to the catalog's places."""
        try:
            names = [str(name) for name in json.loads(msg.data)]
        except (ValueError, TypeError):
            return
        middleman.set_saved_places(names)
        node.get_logger().info(f'Saved places: {", ".join(names) or "none"}.')

    def on_description(msg):
        response_pub.publish(String(data=msg.data))  # speak it

    # Several threads and a reentrant group: a "stop" is handled while a slow request is still with Qwen.
    group = ReentrantCallbackGroup()
    node.create_subscription(String, topic['speech_topic'], on_speech, 10, callback_group=group)
    node.create_subscription(String, topic['done_topic'], on_done, 10, callback_group=group)
    node.create_subscription(String, topic['failed_topic'], on_failed, 10, callback_group=group)
    node.create_subscription(String, topic['status_topic'], on_object_status, 10, callback_group=group)
    node.create_subscription(String, topic['description_topic'], on_description, 10, callback_group=group)
    node.create_subscription(String, topic['following_status_topic'], on_following_status, 10, callback_group=group)
    # navigation's saved places are latched, so the ones from earlier runs arrive as soon as this node starts
    node.create_subscription(String, topic['places_topic'], on_places, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                             callback_group=group)
    import llm
    if not interpreter:
        model = 'off (rules only)'
    elif llm.cloud_available():
        model = f'{llm.CLOUD_MODEL}, the local {llm.LOCAL_MODEL} as fallback'
    else:
        model = f'the local {llm.LOCAL_MODEL} (no API key)'
    node.get_logger().info(f'Middleman ready. Language model: {model}.')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
