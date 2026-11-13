"""ROS 2 node around the middleman.

    /speech/text        (std_msgs/String, in)   what the user said, from speech to text
    /pete/response      (std_msgs/String, out)  what Pete says: replies from the middleman, and the scene description
    /middleman/mode     (std_msgs/String, out)  JSON {"mode", "target", "intent", "stop"} for the mode being started
    /middleman/heard    (std_msgs/String, out)  published when a request goes to perception (no spoken reply); play a beep on it

    /mode, /goal        (std_msgs/String, out)  what perception listens to (autonomy/perception/README.md): an object request
                                                is /goal = the object then /mode = perception, a scene request is /mode =
                                                description, anything else is /mode = the mode's name, or idle
    /object_status      (std_msgs/String, in)   perception's search result, 'found:<object>' or 'not_found:<object>'; a failed
                                                search is spoken ("I couldn't find the chair.") and the mode goes back to idle
    /scene_description  (std_msgs/String, in)   perception's description of the camera view, spoken through /pete/response
    /middleman/done     (std_msgs/String, in)   the robot reached the goal of a location, object or GPS request

Modes other than perception read /middleman/mode. Nothing subscribes to it yet.
Goal modes end when the robot publishes /middleman/done. Following and manual mode have no goal: they end when
the user says "stop", "exit manual mode" or "stop following".

    python3 middleman_node.py [--qwen]
"""

import argparse
import json
import re

from middleman import Middleman, Mode, qwen_interpreter

# The questions perception can answer: it only describes the whole view.
DESCRIBE = re.compile(r"\b(what's|what is|whats|describe)\b.*\b(in front|ahead|around|happening|room)\b|\bwhat (do|can) you see\b")
CANNOT_ANSWER = "Sorry, I can only describe what is in front of me."


def mode_messages(mode, target=''):
    """The (topic, text) messages that put perception into a mode. mode is a Mode's value, or 'idle'."""
    if mode == Mode.OBJECT.value:
        return [('/goal', target), ('/mode', 'perception')]  # the goal first, so the search starts with it
    return [('/mode', mode)]  # perception stays idle for any mode that is not its own


def describe_messages():
    # The description node ignores a mode it is already in, so leave it first to be able to describe again.
    return [('/mode', 'idle'), ('/mode', 'description')]


def main():
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String

    parser = argparse.ArgumentParser()
    parser.add_argument('--qwen', action='store_true', help='let Qwen (through Ollama) read each request')
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('middleman')
    response_pub = node.create_publisher(String, '/pete/response', 10)
    mode_pub = node.create_publisher(String, '/middleman/mode', 10)
    perception = {topic: node.create_publisher(String, topic, 10) for topic in ('/mode', '/goal')}

    interpreter = qwen_interpreter() if args.qwen else None
    if interpreter and not interpreter.warm_up():
        node.get_logger().warning('Cannot reach Ollama, using the rules only.')
        interpreter = None
    ack_pub = node.create_publisher(String, '/middleman/heard', 10)
    middleman = Middleman(interpreter=interpreter, on_ack=lambda: ack_pub.publish(String(data='heard')))

    def send(messages):
        for topic, text in messages:
            perception[topic].publish(String(data=text))

    def announce(mode, target='', intent='', stop=False):
        """Tell everything else that the mode changed: perception on /mode and /goal, the rest on /middleman/mode."""
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
    node.create_subscription(String, '/speech/text', on_speech, 10, callback_group=group)
    node.create_subscription(String, '/middleman/done', on_done, 10, callback_group=group)
    node.create_subscription(String, '/object_status', on_object_status, 10, callback_group=group)
    node.create_subscription(String, '/scene_description', on_description, 10, callback_group=group)
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
