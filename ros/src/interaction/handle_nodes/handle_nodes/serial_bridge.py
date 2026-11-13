import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Bool, UInt8MultiArray


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/handle')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.connection = serial.Serial(port, baud_rate, write_timeout=1)
        self.create_subscription(UInt8MultiArray, '/handle/vibration', self.send_vibration, 1)
        self.held_publisher = self.create_publisher(Bool, '/handle/held', 1)
        self.receive_buffer = b''
        self.create_timer(0.02, self.receive_touch)

    def receive_touch(self):
        available = self.connection.in_waiting
        if not available:
            return

        self.receive_buffer += self.connection.read(available)
        latest_touch = None

        while True:
            start = self.receive_buffer.find(b'<')
            if start == -1:
                self.receive_buffer = b''
                break

            self.receive_buffer = self.receive_buffer[start:]
            end = self.receive_buffer.find(b'>')
            next_start = self.receive_buffer.find(b'<', 1)
            if next_start != -1 and (end == -1 or next_start < end):
                self.receive_buffer = self.receive_buffer[next_start:]
                continue

            if end == -1:
                if len(self.receive_buffer) > 5:
                    self.receive_buffer = b''
                break

            frame = self.receive_buffer[:end + 1]
            self.receive_buffer = self.receive_buffer[end + 1:]
            if frame in (b'<t:0>', b'<t:1>'):
                latest_touch = frame == b'<t:1>'

        if latest_touch is not None:
            message = Bool()
            message.data = latest_touch
            self.held_publisher.publish(message)

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
