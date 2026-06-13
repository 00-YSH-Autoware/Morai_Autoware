#!/usr/bin/env python3
# MORAI /gps(GPSMessage) → /sensing/gnss/.../nav_sat_fix(sensor_msgs/NavSatFix)로 변환하는 노드.
# lat/lon/alt 를 그대로 싣기만 한다. 좌표 정합은 gnss_poser + 맵 map_projector_info 책임.
# morai_ros2_msgs 는 런타임(import)에만 필요 — 브리지 워크스페이스를 오버레이 소싱해야 한다.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from morai_ros2_msgs.msg import GPSMessage


class GpsToNavSatFix(Node):
    def __init__(self):
        super().__init__('gps_to_navsatfix')
        self.input_topic = self.declare_parameter('input_topic', '/gps').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/sensing/gnss/ublox/nav_sat_fix').value
        self.frame_id = self.declare_parameter('frame_id', 'gnss_link').value
        self.stddev_h = self.declare_parameter('position_stddev_horizontal', 1.0).value
        self.stddev_v = self.declare_parameter('position_stddev_vertical', 3.0).value

        # 구독: MORAI /gps 가 어떤 reliability 든 받도록 best_effort(sensor_data).
        self.sub = self.create_subscription(
            GPSMessage, self.input_topic, self.cb, qos_profile_sensor_data)
        # 발행: gnss_poser 의 fix 구독이 RELIABLE(QoS{1}) 이므로 RELIABLE 로 맞춤.
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(NavSatFix, self.output_topic, pub_qos)
        self.get_logger().info(
            f'gps_to_navsatfix: {self.input_topic} -> {self.output_topic} (frame={self.frame_id})')

    def cb(self, msg: GPSMessage):
        fix = NavSatFix()
        fix.header = msg.header
        # gnss_poser 가 frame_id -> base_link TF lookup 을 하므로 안테나 프레임으로 강제.
        fix.header.frame_id = self.frame_id

        # MORAI status 의미는 미문서화 → 매핑하지 않고 sim 은 항상 fix 로 둔다.
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS

        fix.latitude = float(msg.latitude)
        fix.longitude = float(msg.longitude)
        fix.altitude = float(msg.altitude)

        h2 = float(self.stddev_h) ** 2
        v2 = float(self.stddev_v) ** 2
        fix.position_covariance[0] = h2
        fix.position_covariance[4] = h2
        fix.position_covariance[8] = v2
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.pub.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = GpsToNavSatFix()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
