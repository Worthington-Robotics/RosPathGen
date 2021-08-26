from types import new_class
from typing import Optional
import rclpy
from rclpy import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from geometry_msgs.msg import Vector3
from rospathmsgs.msg import Waypoint
import math

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
    x = 0
    y = 0
    z = 0
    def __init__(self, xval, yval, zval = None, speed = 0, name = ""):
        self.x = xval
        self.y = yval
        self.z = zval
        self.speed = speed
        self.name = name

class pathGen(Node):
    pointsInput = [Point]
    pointsOutput = [Point]
    waypointsOutput = [Waypoint]
    maxAccelConst = 8 
    maxVelConst = 10 
    
    def generatepathcallback(self, request, response):
        for i in range(request.points):
            x = request.points[i].point.x
            y = request.points[i].point.y
            if not request.points[i].point.name:
                name = request.points[i].point.name
                newPoint = Point(x, y, name= name)
            else:
                newPoint = Point(x, y)
            self.pointsInput.append(newPoint)
        self.maxAccelConst = request.maxAccel
        self.maxVelConst = request.maxVel
        
        # points into lists of x and y inputs
        for point in self.pointsInput:
            self.xpoints.append(point.x)
            self.ypoints.append(point.y)

        # make a spline for the path
        path = CubicSpline(self.xpoints, self.ypoints)

        for xval in np.arange(self.xpoints[0], self.xpoints[len(self.xpoints) - 1], 0.01):
            yval = path(xval)
            newPoint = Point(xval, yval, name= self.pointsInput[self.xpoints.index(xval)].name)
            self.pointsOutput.append(newPoint)
            
        self.distgraphxs = []
        self.distgraphys = []
        
        self.distgraphxs.append(0)
        self.distgraphys.append(0)

        for point in self.pointsOutput:
            # make the deriv graph loop
            # speed is the deriv of the dist vs time graph

            # Calculate Distances
            distance = math.sqrt(pow((self.distgraphxs[len(self.distgraphxs)-1]-point.x), 2) 
                + pow((self.distgraphys[len(self.distgraphys)-1]-point.y), 2))
            self.distgraphxs = self.pointsOutput.index(point) / 100
            self.distgraphys = distance

        distancegraph = CubicSpline(self.distgraphxs, self.distgraphys)
        derivdistgraph = distancegraph.derivative()
        
        for point in self.pointsOutput:
            newWaypoint = Waypoint(point.x, point.y, speed= derivdistgraph(self.pointsOutput.index(point) / 100), name= point.name)
            self.waypointsOutput.append(newWaypoint)


        
    def __init__(self):
        super().__init__('pathGen')
        self.pathService = self.create_service(GeneratePath, 'generate_path', self.generatepathcallback)
        

def main(args=None):
    rclpy.init(args=args)
    pathgen = pathGen()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
