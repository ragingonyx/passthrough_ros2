import rclpy
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import time

from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class passthrough_pcl(Node):
 
    def __init__(self):
        super().__init__('passthrough_pcl')
       
        self.subscription= self.create_subscription(
            PointCloud2,
            '/points',
            self.listener_callback,
            10)

        self.subscription  # prevent unused variable warning

        self.add_on_set_parameters_callback(self.parameter_event_callback)

        self.declare_parameters(
                                namespace='',
                                parameters = [
                                    ("filter_radius", 3.5), # The minimum distance from the origin for  filtered points, only used is use_radius is True
                                    ("round_size", 1), # Rounding size for caching purposes, only used if use_radius is True
                                    ("x_thresh", -999), # The minimum value for x,y,z to be included, used if use_radius is False
                                    ("y_thresh", -999), 
                                    ("z_thresh", -999), 
                                    ("debug", False), 
                                    ("use_radius", True), # Sets passthrough mode
                                    ]
                                )

        self.publisher_ = self.create_publisher(PointCloud2, '/filtered_points', 10)
        self.cache = {}

    def listener_callback(self, msg):
        header = msg.header
        points = pc2.read_points(msg,field_names=['x','y','z'])
        filtered_points = self.passthrough(pointcloud_data=points)
        filtered_cloud = pc2.create_cloud_xyz32(header=header, points=filtered_points)
        self.publisher_.publish(filtered_cloud)
    
    def parameter_event_callback(self, msg):
        for param in msg:
            if param.name == 'filter_radius' and (param.type_ == Parameter.Type.DOUBLE or param.type_ == Parameter.Type.INTEGER):
                self.filter_radius = float(param.value)
            if param.name == 'round_size' and param.type_ == Parameter.Type.INTEGER:
                self.round_size = param.value
            if param.name == 'x_thresh' and (param.type_ == Parameter.Type.DOUBLE or param.type_ == Parameter.Type.INTEGER):
                self.x_thresh = float(param.value)
            if param.name == 'y_thresh' and (param.type_ == Parameter.Type.DOUBLE or param.type_ == Parameter.Type.INTEGER):
                self.y_thresh= float(param.value)
            if param.name == 'z_thresh' and (param.type_ == Parameter.Type.DOUBLE or param.type_ == Parameter.Type.INTEGER):
                self.z_thresh = float(param.value)
            if param.name == 'use_radius' and param.type_ == Parameter.Type.BOOL:
                self.passthrough_mode = param.value
            if param.name == 'debug' and param.type_ == Parameter.Type.BOOL:
                self.passthrough_mode = param.value
        return SetParametersResult(successful=True)

    """
    Filter data from pointcloud.

    @param pointcloud_data: The point cloud points.
    @type  pointcloud_data: list of iterables, i.e. one iterable for each point, with the
                            elements of each iterable being the values of the fields for 
                            that point (in the same order as the fields parameter)
  
    @return: The filtered point cloud.
    @rtype:  L{sensor_msgs.msg.PointCloud2}
    """

    def passthrough(self, pointcloud_data):
        start = time.time()
        if (self.get_parameter("use_radius").value == True):
            filtered_points = []
            for point in pointcloud_data:
                coordinates = []
                for coordinate in point:
                    coordinates.append(round(coordinate, self.round_size))
                rounded = tuple(coordinates)
                c_value = self.cache[rounded] if rounded in self.cache else -1
                if (c_value == -1):
                    a = np.array([point[0], point[1], point[2]])
                    b = np.zeros(3)
                    dist = np.linalg.norm(a-b)
                    self.cache[rounded] = dist
                else:
                    dist = c_value
                if (dist > self.filter_radius):
                    filtered_points.append(point)
        else:
            filtered_points = []
            for point in pointcloud_data:
                if (point[0] > self.y_thresh and point[1] > self.x_thresh and point[2] > self.z_thresh):
                    filtered_points.append(point)
        end = time.time()
        if (self.get_parameter("debug").value == True):
            print("Time consumed in working: ",end - start)
        return filtered_points

def main(args=None):
    rclpy.init(args=args)
    node = passthrough_pcl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()