#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x2 - x1, y2 - y1)


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_calculation(x, y, xt, yt):
    return math.atan2(yt - y, xt - x)



class turtle_control(Node):

    def __init__(self):
        super().__init__("control_turtle")

 
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)

        self.x, self.y, self.theta = 0.0, 0.0, 0.0


        self.omega_max = 1.8      # max turn speed (rad/s)
        self.v_max = 2.0          # max forward speed
        self.align_tol = 0.01     # aligned if |diff| <= ~1.7°
        self.stop_dist = 0.07     # stop within this distance
        self.diff_brake = 0.6     # start turn easing below ~34°
        self.dist_brake = 1.5     # start approach easing inside 1.5 units

        self.ask_new_target()
        self.timer = self.create_timer(0.018, self.control)  # 55 Hz

    def ask_new_target(self):
        self.target_x = float(input("Enter target x: ").strip())
        self.target_y = float(input("Enter target y: ").strip())

    def pose_cb(self, msg: Pose):
        self.x, self.y, self.theta = msg.x, msg.y, msg.theta

    def control(self):

        dst = euclidean_distance(self.x, self.y, self.target_x, self.target_y)
        hd = angle_calculation(self.x, self.y, self.target_x, self.target_y)
        diff = wrap_to_pi(hd - self.theta)

        cmd = Twist()

        if dst <= self.stop_dist:
            self.pub.publish(Twist())
            self.get_logger().info(f"Reached target ({self.target_x:.2f}, {self.target_y:.2f})")
            self.ask_new_target()
            return


        if abs(diff) > self.align_tol:
            ratio = min(abs(diff) / self.diff_brake, 1.0)                
            omega_scale = math.cos((math.pi / 2.0) * (1.0 - ratio))       
            cmd.angular.z = self.omega_max * omega_scale * (1.0 if diff > 0 else -1.0)
            cmd.linear.x = 0.0
            self.pub.publish(cmd)
            return


        ratio_d = min(dst / self.dist_brake, 1.0)                         
        v_scale = math.cos((math.pi / 2.0) * (1.0 - ratio_d))             
        cmd.linear.x = self.v_max * v_scale
        cmd.angular.z = 0.0  # strict rotate-then-go (no correction while moving)

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = turtle_control()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
