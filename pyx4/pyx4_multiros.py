import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class Robot(Node):

    def __init__(self):
        super().__init__('robot')
        self.robot_name = 'robot_1'
        self.robot_friend = 'robot_2'

        self.robot_publisher_ = self.create_publisher(String, f'{self.robot_name}/connect', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.robot_subscription = self.create_subscription(
            String,
            f'{self.robot_friend}/connect',
            self.listener_callback,
            10)
        self.robot_subscription 

    def timer_callback(self):
        msg = String()
        msg.data = f'[{self.robot_name}] connected: %d' % self.i
        self.robot_publisher_.publish(msg)
        # self.get_logger().info(msg.data)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)

    robot = Robot()

    rclpy.spin(robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()