"""ROS 2 node around the middleman.

    /speech/text      (std_msgs/String, in)   what the user said, from speech to text
    /pete/command     (std_msgs/String, out)  text about what the camera sees, straight to perception (Flavio's node)
    /pete/response    (std_msgs/String, out)  what Pete says: replies from the middleman (perception also publishes here)
    /middleman/mode   (std_msgs/String, out)  JSON {"mode", "target", "intent", "stop"} for the mode being started
    /middleman/heard  (std_msgs/String, out)  published when a request goes to perception (no spoken reply); play a beep on it

    /middleman/done   (std_msgs/String, in)   the robot reached the goal of a location, object or GPS request

Modes other than perception read /middleman/mode. Nothing subscribes to it yet.
Goal modes end when the robot publishes /middleman/done. Following and manual mode have no goal: they end when
the user says "stop", "exit manual mode" or "stop following".

    python3 middleman_node.py [--qwen]
"""

import argparse
import json

from middleman import Middleman, qwen_interpreter


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
    command_pub = node.create_publisher(String, '/pete/command', 10)
    response_pub = node.create_publisher(String, '/pete/response', 10)
    mode_pub = node.create_publisher(String, '/middleman/mode', 10)

    interpreter = qwen_interpreter() if args.qwen else None
    if interpreter and not interpreter.warm_up():
        node.get_logger().warning('Cannot reach Ollama, using the rules only.')
        interpreter = None
    ack_pub = node.create_publisher(String, '/middleman/heard', 10)
    middleman = Middleman(interpreter=interpreter, on_ack=lambda: ack_pub.publish(String(data='heard')))

    def on_speech(msg):
        decision = middleman.handle(msg.data)
        if decision is None:
            return
        if decision.forward:
            command_pub.publish(String(data=decision.forward))  # describe / question: perception answers
        if decision.stop:
            command_pub.publish(String(data='stop'))  # perception stops guiding too
        if decision.mode is not None:
            mode = 'stop' if decision.stop else decision.mode.value
            mode_pub.publish(String(data=json.dumps({'mode': mode, 'target': decision.target, 'intent': decision.intent, 'stop': decision.stop})))
        if decision.reply:
            response_pub.publish(String(data=decision.reply))

    def on_done(msg):
        """Whoever drives a location / object / GPS goal says it has arrived. Following and manual have no goal."""
        middleman.finish()
        mode_pub.publish(String(data=json.dumps({'mode': 'idle', 'target': '', 'intent': '', 'stop': False})))
        response_pub.publish(String(data='Arrived.'))

    # Several threads and a reentrant group: a "stop" is handled while a slow request is still with Qwen.
    group = ReentrantCallbackGroup()
    node.create_subscription(String, '/speech/text', on_speech, 10, callback_group=group)
    node.create_subscription(String, '/middleman/done', on_done, 10, callback_group=group)
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
