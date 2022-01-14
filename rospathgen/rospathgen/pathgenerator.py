from math import e, floor
import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
from scipy.optimize import curve_fit
import sys
import os
import math


def linearSpline(x, a, b):
    return a*x + b

def cubicSpline(x, a, b, c, d):
   return a*x**3 + b*x**2 + c*x + d


class pathGen(Node): 
    def findDistance(self, start: Waypoint, endx, endy):
        return math.sqrt((endx - start.point.x)**2 + (endy - start.point.y)**2)
    
    def generatepathcallback(self, request, response):
        pointsInput = [] # Input list of points
        pointsOutput = [] # Output list of points
        maxAccel = 8 # Max Acceleration (Default)
        maxVelocity = 10 # Max Velocity (Default)
        # X and Y Seperated Input Values
        xvalues = []
        yvalues = []
        timeParameterizedPoints = [] # List of points, linked to list of times
        timeParameterizedTimes = []  # List of times
        linear = False # is the path linear?
        """Give a list of waypoints, gives back entire path"""
        try: 
            #print(request)
            print(request.points)
            pointsInput = request.points
            for waypoint in range(len(pointsInput)):
                x = pointsInput[waypoint].point.x
                y = pointsInput[waypoint].point.y
                # points into lists of x and y inputs
                xvalues.append(x)
                yvalues.append(y)
            # Pull Constants from request
            maxAccel = request.max_accel   
            maxVelocity = request.max_velocity
            maxAngularVelocity = request.max_angular_vel  

            # DESIGN STANDARD: Any Start or End Point should have a velocity of 0
            pointsInput[0].velocity = 0.0
            pointsInput[len(pointsInput)-1].velocity = 0.0

            # Scipy implementation of Curve Fitting
            if len(pointsInput) < 3:
                slope = ((yvalues[1]-yvalues[0])/(xvalues[1]-xvalues[0]))
                print(slope)
                # y-mx = b
                intercept = yvalues[0]-(slope*xvalues[0])
                constants = slope, intercept
                print(intercept)
                linear = True
            else:
                if len(xvalues) < 4:
                    xvalue = xvalues[0]+xvalues[1]/2
                    yvalue = yvalues[0]+yvalues[1]/2
                    xvalues.insert(1, xvalue)
                    yvalues.insert(1, yvalue)
                constants, throwaway = curve_fit(cubicSpline, xvalues, yvalues) # throwaway there so things don't fail
                a,b,c,d = constants 


            # go through a loop where each point is at 0.01 meters of distance (x and y) !! apart            
            for point in np.arange(start=pointsInput[0].point.x, stop=pointsInput[len(pointsInput)-1], step=0.01):
                ...


            # nextxvalue = previousxvalue + previousVelocity*tdelta + maxAccel*tdelta^2                
            # Enforce global max velocity and max reachable velocity by global acceleration limit.
            # vf = sqrt(vi^2 + 2*a*d)

            # for point in reversed(timeParameterizedPoints):
            #     velocityVal = math.sqrt(previousVelocity**2 + 2*-segmentAccel*(self.findDistance(prevWaypoint, xval, yval)))

            # for point in timeParameterizedPoints:
            #     pointsOutput.append(point)    
            response.waypoints = pointsOutput
            return response
        except Exception as e:
            print(e) 
            print("i did a dumb")
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            return response

        
    def __init__(self):
        super().__init__('pathGen')
        self.pathService = self.create_service(GeneratePath, 'generate_path', self.generatepathcallback)
        
# Run a node, don't try to understand
def main(args=None):
    rclpy.init(args=args)
    pathgen = pathGen()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
