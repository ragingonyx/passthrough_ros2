import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2,PointField
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
        header = msg.header

        points = read_points(msg,field_names=['x','y','z'])
        
        filtered_points = []
        for point in points:
            if (point[0] > 2):
                filtered_points.append(point)
        
        
        filtered_cloud = create_cloud_xyz32(header=header, points=filtered_points)

        self.publisher_.publish(filtered_cloud)
        

                

            

           


def main(args=None):
    rclpy.init(args=args)
    node = passthrough_pcl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()