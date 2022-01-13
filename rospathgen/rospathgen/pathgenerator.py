from math import floor
import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
from scipy.optimize import curve_fit
import sys
import os


def linearSpline(x, a, b):
    return a*x + b

def cubicSpline(x, a, b, c, d):
   return a * x^3 + b*x^2 + c*x + d

class pathGen(Node): 

    def generatepathcallback(self, request, response):
        pointsInput = [] # Input list of points
        pointsOutput = [] # Output list of points
        maxAccel = 8 # Max Acceleration (Default)
        maxVelocity = 10 # Max Velocity (Default)
        # X and Y Seperated Input Values
        xvalues = []
        yvalues = []
        linear = False # is the path linear?
        """Give a list of waypoints, gives back entire path"""
        try: 
            #print(request)
            print(request.points)
            pointsInput = request.points
            for point in range(len(pointsInput)):
                x = pointsInput[point].point.x
                y = pointsInput[point].point.y
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
                slope = (yvalues[1]-yvalues[0]/xvalues[1]-xvalues[0])
                #print(slope)
                constants = slope, (yvalues[0]-(xvalues[0]*slope))
                #print(yvalues[0]-(xvalues[0]*slope))
                linear = True
            else:
                constants, _ = curve_fit(cubicSpline, xvalues, yvalues)
                a,b,c,d = constants

            # Velocity "Curve Fit"
            # Every velocity is it's own max velocity, 
            # basically slope at the max acceleration to the given velocity, 
            # then hold until either next velocity is 0 or max velocity changes            
            
            time = 0
            currentAccell = maxAccel

            previousSpeed = 0


            # Timestep is 0.01 seconds        
            # Step = point.velocity*0.01 + maxAccel*0.01
            
            for point in pointsInput: # for every point in the list of input points
                if pointsInput.index(point) == 0: continue
                prevPoint = pointsInput[pointsInput.index(point)-1]
                # For this segment, make a list of points between this point and its previous point with distances at a timestep of 0.01 seconds
                segmentTimedList = np.linspace(int(prevPoint.point.x), int(point.point.x), int(prevPoint.velocity*0.01 + maxAccel*0.01))
                if point.velocity >= prevPoint.velocity:
                    # If increasing in velocity over the segment
                    prevVelocity = prevPoint.velocity
                    for xval in segmentTimedList:
                        # If the point is on input list, immigrate data.
                        if (xvalues.count(xval) != 0):
                            index = xvalues.index(xval)
                            yval = yvalues[index]
                            pointname = pointsInput[index].point_name
                            # Create a point, if no name input, name = empty string so no errors
                            newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                                heading= pointsInput[index].heading, 
                                                max_vel= pointsInput[index].velocity, 
                                                point_name= pointname if pointname is not None else "")
                            prevVelocity = pointsInput[index].velocity
                        else:
                            # If not on list, make a point
                            yval = (float(cubicSpline(xval, *constants)) if not linear else float(linearSpline(xval, *constants)))
                            newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                                heading= 0.0, # TODO Add heading enforcement
                                                max_vel= prevVelocity + maxAccel*0.01,
                                                point_name= "")
                            prevVelocity = prevVelocity + maxAccel*0.01
                
                # Regardless, append the point created to the list
                pointsOutput.append(newPoint)
            



            # 
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
