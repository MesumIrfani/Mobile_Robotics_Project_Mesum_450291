import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.desired_distance_from_wall = 0.5  # meters
        self.current_distance_from_wall = 0.0
        self.error = 0.0
        self.previous_error = 0.0
        self.integral = 0.0
        self.derivative = 0.0

        self.kp = 0.5  # Proportional gain
        self.ki = 0.001  # Integral gain
        self.kd = 0.05  # Derivative gain

        self.linear_speed = 0.1  # Forward speed
        self.obstacle_distance_threshold = 0.4  # meters

        self.obstacle_detected = False

    def scan_callback(self, msg):
        # Assuming the wall is on the right side of the robot (90 degrees to the right)
        self.current_distance_from_wall = msg.ranges[270]  # Angle at 270 degrees

        # Check for obstacles in front (angles around 0 degrees)
        front_distances = msg.ranges[0:10] + msg.ranges[350:360]
        min_front_distance = min(front_distances)

        self.obstacle_detected = min_front_distance < self.obstacle_distance_threshold

    def control_loop(self):
        msg = Twist()

        if self.obstacle_detected:
            msg.linear.x = 0.0
            msg.angular.z = 0.3  # Turn to avoid obstacle
        else:
            self.error = self.desired_distance_from_wall - self.current_distance_from_wall
            self.integral += self.error
            self.derivative = self.error - self.previous_error

            control_signal = self.kp * self.error + self.ki * self.integral + self.kd * self.derivative
            self.previous_error = self.error

            msg.linear.x = self.linear_speed
            msg.angular.z = -control_signal  # Correcting direction to maintain the distance

        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

