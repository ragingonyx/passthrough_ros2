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
        
        filtered_points = self.passthrough(pointcloud_data=points, y_thresh=2)
        
        filtered_cloud = create_cloud_xyz32(header=header, points=filtered_points)

        self.publisher_.publish(filtered_cloud)
        
    
    """
    Filter data from pointcloud.

    @param points: The point cloud points.
    @type  points: list of iterables, i.e. one iterable for each point, with the
                   elements of each iterable being the values of the fields for 
                   that point (in the same order as the fields parameter)
    @param x_thresh: The minimum value for x to be included in filtered cloud.
    @type  x_thresh: float
    @param y_thresh: The minimum value for y to be included in filtered cloud.
    @type  y_thresh: float
    @param z_thresh: The minimum value for z to be included in filtered cloud.
    @type  z_thresh: float
    
    @return: The filtered point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """
    
    def passthrough(self, pointcloud_data, x_thresh=-999, y_thresh=-999, z_thresh=-999):
        filtered_points = []

        for point in pointcloud_data:
            if (point[0] > y_thresh and point[1] > x_thresh and point[2] > z_thresh):
                filtered_points.append(point)
                
        return filtered_points
        
                

            

           


def main(args=None):
    rclpy.init(args=args)
    node = passthrough_pcl()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()