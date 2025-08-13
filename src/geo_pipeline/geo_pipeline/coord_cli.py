import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def ask_float(prompt, lo=None, hi=None):
    while True:
        try:
            s = input(prompt).strip()
            if s.lower() in ('q', 'quit', 'exit'):
                return None
            v = float(s)
            if lo is not None and v < lo:
                print(f"Value must be ≥ {lo}. Try again or type 'q' to quit.")
                continue
            if hi is not None and v > hi:
                print(f"Value must be ≤ {hi}. Try again or type 'q' to quit.")
                continue
            return v
        except ValueError:
            print("Not a number. Try again or type 'q' to quit.")

class CoordCli(Node):
    def __init__(self):
        super().__init__('coord_cli')

        # QoS: keep only the latest message, best-effort delivery
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        self.pub = self.create_publisher(Float32MultiArray, 'coords_in', qos)
        print("\nInteractive coordinate feeder for /coords_in (Float32MultiArray)")
   

    def run_loop(self):
        while rclpy.ok():
            lat1 = ask_float("lat1 (-90..90): ", -90.0, 90.0)
            if lat1 is None: break
            lon1 = ask_float("lon1 (-180..180): ", -180.0, 180.0)
            if lon1 is None: break
            lat2 = ask_float("lat2 (-90..90): ", -90.0, 90.0)
            if lat2 is None: break
            lon2 = ask_float("lon2 (-180..180): ", -180.0, 180.0)
            if lon2 is None: break

            msg = Float32MultiArray()
            # Cast to float32-compatible Python floats
            msg.data = [float(lat1), float(lon1), float(lat2), float(lon2)]
            self.pub.publish(msg)
            print(f"→ Published to /coords_in: {msg.data}\n")

def main():
    rclpy.init()
    node = CoordCli()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

