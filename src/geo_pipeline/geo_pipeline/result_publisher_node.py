import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

class ResultPublisherNode(Node):
    def __init__(self):
        super().__init__('result_publisher')
        self.sub = self.create_subscription(Float32MultiArray, 'geo_out', self.cb, 10)
        self.pub = self.create_publisher(String, 'geo_out_text', 10)
        self.get_logger().info('result_publisher node up. Subscribing to /geo_out')

    def cb(self, msg: Float32MultiArray):
        if len(msg.data) != 2:
            self.get_logger().warn('geo_out must have exactly 2 numbers: [distance_m, bearing_deg]')
            return
        distance_m, bearing_deg = msg.data
        s = String()
        s.data = f'Distance: {distance_m:.2f} m | Initial bearing: {bearing_deg:.2f}°'
        self.pub.publish(s)
        self.get_logger().info(s.data)

def main():
    rclpy.init()
    node = ResultPublisherNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
