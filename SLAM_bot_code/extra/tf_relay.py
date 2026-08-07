import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

# TF REQUIRES depth=100 for proper buffering (NOT 10!)
# This is why view_frames was showing buffer_length: 0.0
TF_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,  # ← Critical: was 10, now 100
)

class TFRelay(Node):
    def __init__(self):
        super().__init__('tf_relay')
        # Subscribe to a separate topic your ESP32 publishes to
        self.sub = self.create_subscription(
            TransformStamped,
            '/tf_raw',        # ← rename ESP32's TF topic to this
            self.cb, TF_QOS)
        self.pub = self.create_publisher(
            TFMessage, '/tf', TF_QOS)
        self.get_logger().info('TF relay running...')

    def cb(self, msg):
        tf_msg = TFMessage()
        tf_msg.transforms = [msg]
        self.pub.publish(tf_msg)

def main():
    rclpy.init()
    rclpy.spin(TFRelay())

if __name__ == '__main__':
    main()