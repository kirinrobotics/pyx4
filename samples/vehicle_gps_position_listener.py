import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy
from px4_msgs.msg import SensorGps

class VehicleGpsPositionListener(Node):

    def __init__(self):
        super().__init__('vehicle_global_position_listener')

        qos = QoSProfile(
            history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )

        self.subscription = self.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos
        )

    def gps_callback(self, msg):
        print("\n" * 10)
        print("RECEIVED VEHICLE GPS POSITION DATA")
        print("==================================")
        print(f"ts: {msg.timestamp}")
        print(f"lat: {msg.lat}")
        print(f"lon: {msg.lon}")
        print(f"alt: {msg.alt}")
        print(f"alt_ellipsoid: {msg.alt_ellipsoid}")
        print(f"s_variance_m_s: {msg.s_variance_m_s}")
        print(f"c_variance_rad: {msg.c_variance_rad}")
        print(f"fix_type: {msg.fix_type}")
        print(f"eph: {msg.eph}")
        print(f"epv: {msg.epv}")
        print(f"hdop: {msg.hdop}")
        print(f"vdop: {msg.vdop}")
        print(f"noise_per_ms: {msg.noise_per_ms}")
        print(f"vel_m_s: {msg.vel_m_s}")
        print(f"vel_n_m_s: {msg.vel_n_m_s}")
        print(f"vel_e_m_s: {msg.vel_e_m_s}")
        print(f"vel_d_m_s: {msg.vel_d_m_s}")
        print(f"cog_rad: {msg.cog_rad}")
        print(f"vel_ned_valid: {msg.vel_ned_valid}")
        print(f"timestamp_time_relative: {msg.timestamp_time_relative}")
        print(f"time_utc_usec: {msg.time_utc_usec}")
        print(f"satellites_used: {msg.satellites_used}")
        print(f"heading: {msg.heading}")
        print(f"heading_offset: {msg.heading_offset}")

def main(args=None):
    print("Starting vehicle_global_position listener node...")
    rclpy.init(args=args)
    vehicle_gps_position_listener = VehicleGpsPositionListener()
    rclpy.spin(vehicle_gps_position_listener)
    vehicle_gps_position_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()