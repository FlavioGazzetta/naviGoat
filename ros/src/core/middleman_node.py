"""ROS 2 node around the middleman. Run it with launch/middleman.launch.py. The README in this folder has the topics table.

    speech_topic         /speech/text         in   what the user said, from speech to text
    response_topic       /pete/response       out  what Pete says: replies from the middleman, and the scene description
    middleman_mode_topic /middleman/mode      out  JSON {"mode", "target", "intent", "stop"} for the mode being started
    heard_topic          /middleman/heard     out  published when a request goes to perception (no spoken reply); play a beep on it
    mode_topic, goal_topic  /mode, /goal      out  what perception listens to (autonomy/perception/README.md): an object request
                                                   is /goal = the object then /mode = perception, a scene request is /mode =
                                                   description, anything else is /mode = the mode's name, or idle
    status_topic         /object_status       in   perception's search result, 'found:<object>' or 'not_found:<object>'; a failed
                                                   search is spoken ("I couldn't find the chair.") and the mode goes back to idle
    description_topic    /scene_description   in   perception's description of the camera view, spoken through the response topic
    done_topic           /middleman/done      in   the robot reached the goal of a location, object or GPS request

Every topic name above is a ROS parameter (the name in the first column, the default in the second), and so is use_qwen.

Modes other than perception read /middleman/mode. Nothing subscribes to it yet.
Goal modes end when the robot publishes /middleman/done. Following and manual mode have no goal: they end when
the user says "stop", "exit manual mode" or "stop following".

    python3 middleman_node.py --ros-args -p use_qwen:=true
"""

import json
import re

from middleman import Middleman, Mode, qwen_interpreter

# The topic parameters and their defaults. mode_topic and goal_topic are the names perception listens to.
TOPICS = {
    'speech_topic': '/speech/text', 'response_topic': '/pete/response', 'middleman_mode_topic': '/middleman/mode',
    'heard_topic': '/middleman/heard', 'mode_topic': '/mode', 'goal_topic': '/goal', 'status_topic': '/object_status',
    'description_topic': '/scene_description', 'done_topic': '/middleman/done'}

# The questions perception can answer: it only describes the whole view.
DESCRIBE = re.compile(r"\b(what's|what is|whats|describe)\b.*\b(in front|ahead|around|happening|room)\b|\bwhat (do|can) you see\b"
                      r"|\bdescribe\b|\bwhere am i\b|\blook around\b")
CANNOT_ANSWER = "Sorry, I can only describe what is in front of me."


def mode_messages(mode, target=''):
    """The (topic parameter, text) messages that put perception into a mode. mode is a Mode's value, or 'idle'."""
    if mode == Mode.OBJECT.value:
        return [('goal_topic', target), ('mode_topic', 'perception')]  # the goal first, so the search starts with it
    return [('mode_topic', mode)]  # perception stays idle for any mode that is not its own


def describe_messages():
    # The description node ignores a mode it is already in, so leave it first to be able to describe again.
    return [('mode_topic', 'idle'), ('mode_topic', 'description')]


def main():
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node('middleman')
    param = lambda name, default: node.declare_parameter(name, default).value
    topic = {name: param(name, default) for name, default in TOPICS.items()}
    use_qwen = param('use_qwen', False)  # let Qwen (through Ollama) match a need like "I'm hungry" to a known place

    response_pub = node.create_publisher(String, topic['response_topic'], 10)
    mode_pub = node.create_publisher(String, topic['middleman_mode_topic'], 10)
    ack_pub = node.create_publisher(String, topic['heard_topic'], 10)
    perception = {name: node.create_publisher(String, topic[name], 10) for name in ('mode_topic', 'goal_topic')}

    interpreter = qwen_interpreter() if use_qwen else None
    if interpreter and not interpreter.warm_up():
        node.get_logger().warning('Cannot reach Ollama, using the rules only.')
        interpreter = None
    middleman = Middleman(interpreter=interpreter, on_ack=lambda: ack_pub.publish(String(data='heard')))

    def send(messages):
        for name, text in messages:
            perception[name].publish(String(data=text))

    def announce(mode, target='', intent='', stop=False):
        """Tell everything else that the mode changed: perception on the mode and goal topics, the rest on the middleman one."""
        send(mode_messages(mode, target))
        mode_pub.publish(String(data=json.dumps({'mode': 'stop' if stop else mode, 'target': target, 'intent': intent, 'stop': stop})))

    def on_speech(msg):
        decision = middleman.handle(msg.data)
        if decision is None:
            return
        if decision.forward:  # a question about what the camera sees
            if DESCRIBE.search(decision.forward):
                send(describe_messages())
            else:
                response_pub.publish(String(data=CANNOT_ANSWER))
        if decision.stop:
            announce('idle', stop=True)
        elif decision.mode is not None:
            announce(decision.mode.value, decision.target, decision.intent)
        if decision.reply:
            response_pub.publish(String(data=decision.reply))

    def on_done(msg):
        """Whoever drives a location / object / GPS goal says it has arrived. Following and manual have no goal."""
        middleman.finish()
        announce('idle')
        response_pub.publish(String(data='Arrived.'))

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

    def on_description(msg):
        response_pub.publish(String(data=msg.data))  # speak it

    # Several threads and a reentrant group: a "stop" is handled while a slow request is still with Qwen.
    group = ReentrantCallbackGroup()
    node.create_subscription(String, topic['speech_topic'], on_speech, 10, callback_group=group)
    node.create_subscription(String, topic['done_topic'], on_done, 10, callback_group=group)
    node.create_subscription(String, topic['status_topic'], on_object_status, 10, callback_group=group)
    node.create_subscription(String, topic['description_topic'], on_description, 10, callback_group=group)
    node.get_logger().info(f"Middleman ready. Qwen is {'on' if interpreter else 'off'}.")
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
