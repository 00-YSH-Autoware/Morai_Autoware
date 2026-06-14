#!/usr/bin/env python3
# MORAI /lidar/points(PointXYZI) → PointXYZIRC 로 변환 (Autoware crop box 가 요구하는 레이아웃).
# 최신 autoware_pointcloud_preprocessor 는 PointXYZIRC(ring/channel 포함)만 받고 XYZI 면 abort 한다.
# CARLA 는 인터페이스가 직접 XYZIRC 로 publish 하는데, MORAI 는 raw XYZI 라 외부에서 변환해 준다.
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

# ROS PointField datatype → numpy dtype
_NP = {
    PointField.INT8: 'i1', PointField.UINT8: 'u1',
    PointField.INT16: 'i2', PointField.UINT16: 'u2',
    PointField.INT32: 'i4', PointField.UINT32: 'u4',
    PointField.FLOAT32: 'f4', PointField.FLOAT64: 'f8',
}

# PointXYZIRC: x,y,z float32 / intensity u8 / return_type u8 / channel u16 (point_step 16)
_OUT_DT = np.dtype({
    'names': ['x', 'y', 'z', 'intensity', 'return_type', 'channel'],
    'formats': ['<f4', '<f4', '<f4', 'u1', 'u1', '<u2'],
    'offsets': [0, 4, 8, 12, 13, 14],
    'itemsize': 16,
})
_OUT_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.UINT8, count=1),
    PointField(name='return_type', offset=13, datatype=PointField.UINT8, count=1),
    PointField(name='channel', offset=14, datatype=PointField.UINT16, count=1),
]


class XyziToXyzirc(Node):
    def __init__(self):
        super().__init__('pointcloud_xyzi_to_xyzirc')
        self.input_topic = self.declare_parameter('input_topic', '/lidar/points').value
        self.output_topic = self.declare_parameter(
            'output_topic', '/sensing/lidar/top/pointcloud_before_sync').value

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self.cb, qos_profile_sensor_data)
        # crop box 구독과 호환되도록 RELIABLE 로 발행 (best_effort 면 안 받는 경우 있음).
        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=5)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.get_logger().info(
            f'xyzi->xyzirc: {self.input_topic} -> {self.output_topic}')

    def cb(self, msg: PointCloud2):
        try:
            in_dt = np.dtype({
                'names': [f.name for f in msg.fields],
                'formats': [_NP[f.datatype] for f in msg.fields],
                'offsets': [f.offset for f in msg.fields],
                'itemsize': msg.point_step,
            })
        except KeyError:
            self.get_logger().warn('unsupported field datatype, skip',
                                   throttle_duration_sec=5.0)
            return

        arr = np.frombuffer(msg.data, dtype=in_dt)
        names = arr.dtype.names
        n = arr.shape[0]

        out = np.zeros(n, dtype=_OUT_DT)
        out['x'] = arr['x']
        out['y'] = arr['y']
        out['z'] = arr['z']
        if 'intensity' in names:
            out['intensity'] = np.clip(arr['intensity'], 0, 255).astype('u1')
        # return_type, channel 은 MORAI 에 정보 없음 → 0

        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = n
        out_msg.fields = _OUT_FIELDS
        out_msg.is_bigendian = False
        out_msg.point_step = 16
        out_msg.row_step = 16 * n
        out_msg.is_dense = msg.is_dense
        out_msg.data = out.tobytes()
        self.pub.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = XyziToXyzirc()
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
