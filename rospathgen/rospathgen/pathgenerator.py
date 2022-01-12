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
        pointsInput = []
        pointsOutput = []
        maxAccel = 8 
        maxVelConst = 10 
        xvalues = []
        yvalues = []
        linear = False
        """Give a list of waypoints, gives back entire path"""
        try: 
            print(request)
            print(request.points)
            pointsInput = request.points
            for point in range(len(request.points)):
                x = request.points[point].point.x
                y = request.points[point].point.y
                # points into lists of x and y inputs
                xvalues.append(x)
                yvalues.append(y)
            maxAccel = request.max_accel     

            # Scipy implementation
            if len(pointsInput) < 3:
                slope = (yvalues[1]-yvalues[0]/xvalues[1]-xvalues[0])
                #print(slope)
                constants = slope, (yvalues[0]-(xvalues[0]*slope))
                #print(yvalues[0]-(xvalues[0]*slope))
                linear = True
            else:
                constants, _ = curve_fit(cubicSpline, xvalues, yvalues)
                a,b,c,d = constants

            for value in xvalues:
                if xvalues.index(value) == 0: continue
                for xval in np.linspace(xvalues[xvalues.index(value)-1], value, floor(abs((value-value-1)/0.05))):
                    # If the xpoint is on the list, make it a point, immigrate data.
                    #print(xval)
                    if (xvalues.count(xval) != 0):
                        index = xvalues.index(xval)
                        yval = yvalues[index]
                        pointname = pointsInput[index].point_name
                        # Create a point, if no name input, name = empty string so no errors
                        newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                            heading= pointsInput[index].heading, 
                                            max_vel= pointsInput[index].max_vel, 
                                            point_name= pointname if pointname is not None else "")
                        currentMaxSpeed = pointsInput[index].max_vel
                    else:
                        # If not on list, make a point
                        yval = (float(cubicSpline(xval, *constants)) if not linear else float(linearSpline(xval, *constants)))
                        newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                            heading= 0.0, # TODO Add heading enforcement
                                            max_vel= currentMaxSpeed, 
                                            point_name= "")
                    pointsOutput.append(newPoint)
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
