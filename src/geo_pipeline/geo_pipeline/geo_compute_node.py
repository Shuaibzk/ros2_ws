import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy 

R_EARTH_M = 6371000.0

def deg2rad(d):
    return d * math.pi / 180.0

def rad2deg(r):
    return r * 180.0 / math.pi

def haversine_distance_m(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1, lon1, lat2, lon2 = map(deg2rad, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R_EARTH_M * c

def initial_bearing_deg(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    lat1, lon1, lat2, lon2 = map(deg2rad, [lat1_deg, lon1_deg, lat2_deg, lon2_deg])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1)*math.sin(lat2) - math.sin(lat1)*math.cos(lat2)*math.cos(dlon)
    theta = math.atan2(x, y)
    return (rad2deg(theta) + 360.0) % 360.0

class GeoComputeNode(Node):
    def __init__(self):
        super().__init__('geo_compute')
        qos = QoSProfile(                              
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.sub = self.create_subscription(                 
            Float32MultiArray, 'coords_in', self.cb, qos
        )
        self.pub = self.create_publisher(Float32MultiArray, 'geo_out', 10)
        self.get_logger().info('geo_compute up. Send [lat1, lon1, lat2, lon2] to /coords_in')

    def cb(self, msg: Float32MultiArray):
        if len(msg.data) != 4:
            self.get_logger().warn('coords_in needs exactly 4 numbers: [lat1, lon1, lat2, lon2]')
            return
        lat1, lon1, lat2, lon2 = msg.data
        dist = haversine_distance_m(lat1, lon1, lat2, lon2)
        brng = initial_bearing_deg(lat1, lon1, lat2, lon2)
        out = Float32MultiArray()
        out.data = [dist, brng]
        self.pub.publish(out)
        self.get_logger().info(f'distance={dist:.2f} m, bearing={brng:.2f}°')

def main():
    rclpy.init()
    node = GeoComputeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

