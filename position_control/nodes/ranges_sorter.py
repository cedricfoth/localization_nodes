#!/usr/bin/env python3

import rclpy
from hippo_msgs.msg import RangeMeasurement, RangeMeasurementArray
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

class RangesSorter(Node):
    def __init__(self):
        super().__init__(node_name='ranges_sorter')
        
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)
        
        self.ranges_sub = self.create_subscription(
            msg_type=RangeMeasurementArray,
            topic='ranges',
            callback=self.on_ranges,
            qos_profile=qos)
        
        self.ranges_sorted_pub = self.create_publisher(
            msg_type=RangeMeasurementArray,
            topic='ranges_sorted',
            qos_profile=qos
        )

    def on_ranges(self, msg: RangeMeasurementArray):
        new_msg = RangeMeasurementArray()
        now = self.get_clock().now()
        measurement: RangeMeasurement

        new_msg.measurements = [RangeMeasurement(), RangeMeasurement(), RangeMeasurement(), RangeMeasurement()]

        # print(new_msg.measurements)

        for measurement in msg.measurements:
            id = measurement.id
            # print(id)
            new_msg.measurements[id].range = measurement.range
            new_msg.measurements[id].header.stamp = now.to_msg()
            new_msg.measurements[id].id = id

        self.ranges_sorted_pub.publish(new_msg)


def main():
    rclpy.init()
    node = RangesSorter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
