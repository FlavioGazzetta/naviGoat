import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import UInt8MultiArray


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/handle')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.connection = serial.Serial(port, baud_rate, write_timeout=1)
        self.create_subscription(UInt8MultiArray, '/handle/vibration', self.send_vibration, 1)

    def send_vibration(self, message):
        if len(message.data) != 2:
            self.get_logger().warning('Expected [side, magnitude] vibration values')
            return

        side, magnitude = message.data
        if side not in (0, 1):
            self.get_logger().warning('Expected side 0 (left) or 1 (right)')
            return

        direction = 'l' if side == 0 else 'r'
        self.connection.write(f'<{direction}:{magnitude}>\n'.encode('ascii'))


def main():
    rclpy.init()
    node = SerialBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.connection.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
