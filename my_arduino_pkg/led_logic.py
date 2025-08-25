import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

class LedLogic(Node):
    def __init__(self):
        super().__init__('led_logic')
        self.sub = self.create_subscription(Int32, 'distance', self.callback, 10)
        self.pub = self.create_publisher(String, 'led_control', 10)

    def callback(self, msg):
        if msg.data < 20:  # if object closer than 20 cm
            out = String()
            out.data = "on"
            self.pub.publish(out)
            self.get_logger().info("LED ON")
        else:
            out = String()
            out.data = "off"
            self.pub.publish(out)
            self.get_logger().info("LED OFF")

def main(args=None):
    rclpy.init(args=args)
    node = LedLogic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

