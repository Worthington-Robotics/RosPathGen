from typing import Optional
import rclpy
from rclpy import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from geometry_msgs.msgs import Vector3
from rospathmsgs.msg import Waypoint

class Point():
    name = ""
    x = 0
    y = 0
    z = 0
    def __init__(self, xval, yval, zval = None, name = ""):
        self.name = name
        self.x = xval
        self.y = yval
        self.z = zval

class Waypoint():
    speed = 0
    name = ""
    def __init__(self, point: Point, speed, name):
        self.point = point
        self.speed = speed
        self.name = name

class pathGen(Node):
    pointInput = [Point]
    pointsOutput = [Point]
    waypointsOutput = [Waypoint]
    
    def __init__(self):
        super().__init__('pathGen')
        self.pathService = self.create_service(GeneratePath, 'generate_path', self.generatepathcallback)

    def generatepathcallback(self, request, response):
        for i in range(request.points):
            # TODO add each point to point input
            filler
        maxAccelConst = request.maxAccel
        maxVelConst = request.maxVel
        
        # points into lists of x and y inputs
        for point in self.pointInput:
            self.xpoints.append(point.x)
            self.ypoints.append(point.y)

        # make a spline
        cs = CubicSpline(self.xpoints, self.ypoints)

        for xval in np.arange(self.xpoints[0], self.xpoints[len(self.xpoints) - 1], 0.01):
            # TODO do the math things and figure out how to store things in the way I want
            # Use the waypoint msg
            filler
        


        



def main(args=None):
    rclpy.init(args=args)
    pathgen = pathGen()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
