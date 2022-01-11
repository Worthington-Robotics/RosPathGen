import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3

class pathTester(Node):
    pointsOutput = [Waypoint]
    maxAccel = 8 
    maxVelConst = 10 
    xpoints = []
    ypoints = []

    
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(pathGen, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response

# Run a node, don't try to understand
def main(args=None):
    rclpy.init(args=args)
    pathgen = pathGen()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
