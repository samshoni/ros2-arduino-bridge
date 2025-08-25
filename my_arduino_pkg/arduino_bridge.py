import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import serial

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.ser = serial.Serial(port, baud, timeout=1)

        # Publisher: distance
        self.pub = self.create_publisher(Int32, 'distance', 10)

        # Subscriber: LED control
        self.sub = self.create_subscription(String, 'led_control', self.led_callback, 10)

        self.timer = self.create_timer(0.2, self.read_serial)

    def read_serial(self):
        if self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("DIST:"):
                try:
                    dist = int(line.split(":")[1])
                    msg = Int32()
                    msg.data = dist
                    self.pub.publish(msg)
                    self.get_logger().info(f"Distance: {dist} cm")
                except:
                    pass

    def led_callback(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ["on", "off"]:
            self.ser.write((cmd + '\n').encode('utf-8'))
            self.get_logger().info(f"Sent to Arduino: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

