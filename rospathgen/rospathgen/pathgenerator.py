import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3

class pathGen(Node):
    waypointsOutput = [Waypoint]
    maxAccelConst = 8 
    maxVelConst = 10 
    
    def generatepathcallback(self, request, response):
        try: 
            for i in len(request.points):
                x = request.points[i].point.x
                y = request.points[i].point.y
                # points into lists of x and y inputs
                self.xpoints.append(x)
                self.ypoints.append(y)   
            self.maxAccelConst = request.max_accel         

            # make a spline for the path
            path = CubicSpline(self.xpoints, self.ypoints)

            currentMaxSpeed = 0
            for xval in np.arange(self.xpoints[0], self.xpoints[len(self.xpoints) - 1], 0.01):
                yval = path(xval)
                if (self.xpoints.index(xval) != -1):
                    pointname = self.pointsInput[self.xpoints.index(xval)].name
                    newPoint = Waypoint(Vector3(xval, yval, 0),  speed= self.pointsInput[self.xpoints.index(xval)].max_vel, name= pointname if pointname is not None else "")
                    currentMaxSpeed = self.pointsInput[self.xpoints.index(xval)].max_vel
                else:
                    newPoint = Waypoint(Vector3(xval, yval, 0), speed = currentMaxSpeed)
                self.pointsOutput.append(newPoint)
            response.waypoints = self.pointsOutput

            return response
        except: 
            return response

        
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
