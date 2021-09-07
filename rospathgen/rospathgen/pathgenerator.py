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

class Waypoint():
    speed = 10
    name = ""
    x = 0
    y = 0
    def __init__(self, xval, yval, speed = 10, name = ""):
        self.x = xval
        self.y = yval
        self.speed = speed
        self.name = name

class pathGen(Node):
    pointsInput = [Waypoint]
    pointsOutput = [Waypoint]
    waypointsOutput = [Waypoint]
    maxAccelConst = 8 
    maxVelConst = 10 
    
    def generatepathcallback(self, request, response):
        for i in range(request.points):
            x = request.points[i].point.x
            y = request.points[i].point.y
            if not request.points[i].point.name:
                name = request.points[i].point.name
                newPoint = Waypoint(x, y, speed =  request.points[i].point.speed, name= name)
            else:
                newPoint = Waypoint(x, y, speed =  request.points[i].point.speed)
            self.pointsInput.append(newPoint)
        self.maxAccelConst = request.maxAccel
        
        # points into lists of x and y inputs
        self.xpoints.append(x)
        self.ypoints.append(y)            

        # make a spline for the path
        path = CubicSpline(self.xpoints, self.ypoints)

        currentMaxSpeed = 0
        for xval in np.arange(self.xpoints[0], self.xpoints[len(self.xpoints) - 1], 0.01):
            yval = path(xval)
            if (self.xpoints.index(xval) != -1):
                newPoint = Waypoint(xval, yval, speed= self.pointsInput[self.xpoints.index(xval)].speed, name= self.pointsInput[self.xpoints.index(xval)].name)
                currentMaxSpeed = self.pointsInput[self.xpoints.index(xval)].speed
            else:
                newPoint = Waypoint(xval, yval, speed = currentMaxSpeed)
            self.pointsOutput.append(newPoint)


        
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
