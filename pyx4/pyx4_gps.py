"""
Python implementation of PX4

"""

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from px4_msgs.msg import SensorGps
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy

class Pyx4_GPS(Node):

    def __init__(self):
        super().__init__('pyx4_gps')

        qos = QoSProfile(
            history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )

        self.gps_subscription = self.create_subscription(
            SensorGps, 
            "/fmu/out/vehicle_gps_position", 
            self.gps_callback, 
            qos)
        self.gps_subscription

    def gps_callback(self, msg):
        print("RECEIVED VEHICLE GPS POSITION DATA")
        print("==================================")
        print("[Lat] : ", msg.lat)
        print("[lon] : ", msg.lon)
        print("[Alt]: ", msg.alt)
        print("[Heading] : ", msg.heading)


def main(args=None):
    rclpy.init(args=args)
    print("Starting infor node...\n")
    infor = Pyx4_GPS()
    rclpy.spin(infor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    infor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
