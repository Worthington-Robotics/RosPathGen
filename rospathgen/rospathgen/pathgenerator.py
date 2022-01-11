import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3

class pathGen(Node):
    pointsOutput = [Waypoint]
    maxAccel = 8 
    maxVelConst = 10 
    xpoints = []
    ypoints = []
    
    def generatepathcallback(self, request, response):
        """Give a list of waypoints, gives back entire path"""
        try: 
            print(request)
            for point in len(request.points):
                x = request.points[point].point.x
                y = request.points[point].point.y
                # points into lists of x and y inputs
                self.xpoints.append(x)
                self.ypoints.append(y)   
            self.maxAccel = request.max_accel         

            # make a spline for the path
            path = CubicSpline(self.xpoints, self.ypoints)

            currentMaxSpeed = 0
            for xval in np.arange(self.xpoints[0], self.xpoints[len(self.xpoints) - 1], 0.01):
                yval = path(xval)
                # If the xpoint is on the list, make it a point, immigrate data.
                if (self.xpoints.index(xval) != -1):
                    pointname = self.pointsInput[self.xpoints.index(xval)].name
                    # Create a point, if no name input, name = empty string so no errors
                    newPoint = (Waypoint(Vector3(xval, yval, 0), speed = self.pointsInput[self.xpoints.index(xval)].max_vel, 
                            name = pointname if pointname is not None else ""))
                    currentMaxSpeed = self.pointsInput[self.xpoints.index(xval)].max_vel
                else:
                    # If not on list, make a point
                    newPoint = Waypoint(Vector3(xval, yval, 0), speed = currentMaxSpeed)
                self.pointsOutput.append(newPoint)
            # Connect created list of points to output
            response.waypoints = self.pointsOutput

            return response
        except: 
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
