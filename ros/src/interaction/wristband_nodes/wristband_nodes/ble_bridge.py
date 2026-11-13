import asyncio

from bleak import BleakClient, BleakScanner
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray


DEVICE_NAME = 'naviGoatWristband'
WRITE_CHARACTERISTIC_UUID = 'TBD'
RETRY_DELAY_SECONDS = 3
DIRECTIONS = ('l', 'r', 'f', 'b')


class BleBridge(Node):
    def __init__(self):
        super().__init__('ble_bridge')
        self.connected = False
        self.latest_frame = None
        self.create_subscription(UInt8MultiArray, '/wristband/vibration', self.send_vibration, 1)

    def send_vibration(self, message):
        if len(message.data) != 2:
            self.get_logger().warning('Expected [side, magnitude] vibration values')
            return

        side, magnitude = message.data
        if side not in range(len(DIRECTIONS)):
            self.get_logger().warning('Expected side 0 (left), 1 (right), 2 (forward) or 3 (back)')
            return

        if not self.connected:
            return

        # send only latest command
        self.latest_frame = f'<{DIRECTIONS[side]}:{magnitude}>'.encode('ascii')

    async def run(self):
        if WRITE_CHARACTERISTIC_UUID == 'TBD': # remove once uuid known
            self.get_logger().error('Wristband write characteristic UUID is TBD')
            return

        while rclpy.ok():
            try:
                device = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=5)
                if device is None:
                    self.get_logger().warning(f'Waiting for {DEVICE_NAME}')
                else:
                    async with BleakClient(device) as client:
                        characteristic = client.services.get_characteristic(WRITE_CHARACTERISTIC_UUID)
                        if characteristic is None:
                            raise RuntimeError(f'Write characteristic {WRITE_CHARACTERISTIC_UUID} not found')

                        response = 'write' in characteristic.properties
                        if not response and 'write-without-response' not in characteristic.properties:
                            raise RuntimeError('Wristband characteristic is not writable')

                        self.latest_frame = None
                        self.connected = True
                        self.get_logger().info(f'Connected to {DEVICE_NAME}')

                        while client.is_connected and rclpy.ok():
                            frame = self.latest_frame
                            self.latest_frame = None
                            if frame is None:
                                await asyncio.sleep(0.05)
                                continue

                            await client.write_gatt_char(characteristic, frame, response=response)

            except Exception as error:
                self.get_logger().warning(f'Wristband BLE connection failed: {error}')
            finally:
                if self.connected:
                    self.get_logger().warning('Wristband disconnected')
                self.connected = False
                self.latest_frame = None

            if rclpy.ok():
                await asyncio.sleep(RETRY_DELAY_SECONDS)


async def run(node):
    ble_task = asyncio.create_task(node.run())
    try:
        while rclpy.ok() and not ble_task.done():
            rclpy.spin_once(node, timeout_sec=0)
            await asyncio.sleep(0.05)
    finally:
        ble_task.cancel()
        try:
            await ble_task
        except asyncio.CancelledError:
            pass

def main():
    rclpy.init()
    node = BleBridge()

    try:
        asyncio.run(run(node))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
