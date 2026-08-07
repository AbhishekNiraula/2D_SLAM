#!/usr/bin/env python3
"""
tf_relay.py
───────────────────────────────────────────────────────────
Bridges /tf_raw (geometry_msgs/TransformStamped from ESP32)
     → /tf     (tf2_msgs/TFMessage that RViz2 + TF tree expect)

ROOT CAUSE OF MISSING odom FRAME:
  The standard /tf topic uses QoS:
    - Reliability:  RELIABLE
    - Durability:   VOLATILE
    - History:      KEEP_LAST (depth 100)
  Publishing with depth=10 default QoS causes the TF buffer in
  rclcpp/rclpy to silently drop the transform, so odom never
  appears as a root in view_frames even though the message flows.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage


# QoS that matches what tf2_ros TransformBroadcaster uses internally
TF_QOS = QoSProfile(
    reliability = QoSReliabilityPolicy.RELIABLE,
    durability  = QoSDurabilityPolicy.VOLATILE,
    history     = QoSHistoryPolicy.KEEP_LAST,
    depth       = 100,
)


class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')

        # Subscribe to raw TransformStamped from ESP32
        self.sub = self.create_subscription(
            TransformStamped,
            '/tf_raw',
            self.cb,
            TF_QOS,
        )

        # Publish as TFMessage on /tf with matching QoS
        self.pub = self.create_publisher(
            TFMessage,
            '/tf',
            TF_QOS,
        )

        self.get_logger().info('TF relay running — bridging /tf_raw → /tf')

    def cb(self, msg: TransformStamped):
        tf_msg = TFMessage()
        tf_msg.transforms = [msg]
        self.pub.publish(tf_msg)
        # Uncomment to debug:
        # self.get_logger().info(
        #     f'Relayed {msg.header.frame_id} → {msg.child_frame_id}')


def main():
    rclpy.init()
    node = TFRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()