#!/usr/bin/env python3
# MORAI /Imu(BEST_EFFORT) 를 RELIABLE 로 중계한다.
# autoware_imu_corrector 의 입력 구독이 RELIABLE 이라, MORAI 의 BEST_EFFORT /Imu 를
# 그대로는 못 받아 imu_data 가 안 나오고 → ekf twist 미갱신 → 측위 공분산 폭증 →
# localization accuracy ERROR → 자율주행 가용 차단으로 번진다. 그래서 QoS 만 다리 놓는다.
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuQosRelay(Node):
    def __init__(self):
        super().__init__('imu_qos_relay')
        self.input_topic = self.declare_parameter('input_topic', '/Imu').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/sensing/imu/imu_raw').value

        # 구독: MORAI 가 BEST_EFFORT 라 best_effort(sensor_data) 로 받음.
        self.sub = self.create_subscription(
            Imu, self.input_topic, self.cb, qos_profile_sensor_data)
        # 발행: imu_corrector 가 RELIABLE 구독이라 RELIABLE 로 맞춤.
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=100)
        self.pub = self.create_publisher(Imu, self.output_topic, pub_qos)
        self.get_logger().info(
            f'imu_qos_relay: {self.input_topic}(best_effort) -> {self.output_topic}(reliable)')

    def cb(self, msg: Imu):
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuQosRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
