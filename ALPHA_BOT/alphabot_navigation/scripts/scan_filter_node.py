#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math

class ScanFilterNode(Node):

    def __init__(self):
        super().__init__('scan_filter_node')

        # Define the square area (in meters) to filter out
        # Adjust these values to match your robot's footprint or exclusion zone
        self.square_x_min = -0.45  # half of 0.9m
        self.square_x_max = 0.45
        self.square_y_min = -0.32  # half of 0.64m
        self.square_y_max = 0.32

        # Subscribe to raw scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.scan_callback,
            10)

        # Publisher for filtered scan
        self.publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.get_logger().info("Scan Filter Node Started. Filtering square area: "
                               f"X:[{self.square_x_min}, {self.square_x_max}], "
                               f"Y:[{self.square_y_min}, {self.square_y_max}]")

    def scan_callback(self, msg):
        # Copy the original message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header
        filtered_scan.angle_min = msg.angle_min
        filtered_scan.angle_max = msg.angle_max
        filtered_scan.angle_increment = msg.angle_increment
        filtered_scan.time_increment = msg.time_increment
        filtered_scan.scan_time = msg.scan_time
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # Prepare new ranges list
        new_ranges = []

        angle = msg.angle_min
        for r in msg.ranges:
            # Check if range is valid
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                new_ranges.append(float('inf'))  # or r, if you want to preserve invalid values
            else:
                # Convert polar (r, angle) to cartesian (x, y)
                x = r * math.cos(angle)
                y = r * math.sin(angle)

                # Check if point is INSIDE the square → then filter it out (set to inf)
                if (self.square_x_min <= x <= self.square_x_max) and \
                   (self.square_y_min <= y <= self.square_y_max):
                    new_ranges.append(float('inf'))  # Mark as no obstacle
                else:
                    new_ranges.append(r)

            angle += msg.angle_increment

        filtered_scan.ranges = new_ranges
        self.publisher.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    