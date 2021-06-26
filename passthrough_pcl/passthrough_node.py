import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from .point_cloud2 import *

class passthrough_pcl(Node):
 
    def __init__(self):
        super().__init__('passthrough_pcl')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/points',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(PointCloud2, '/filtered_points', 10)

    def listener_callback(self, msg):

        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = passthrough_pcl()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()