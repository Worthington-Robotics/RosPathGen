import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from scipy.interpolate import CubicSpline
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3

class pathGen(Node):  
    
    def generatepathcallback(self, request, response):
        pointsInput = []
        pointsOutput = []
        maxAccel = 8 
        maxVelConst = 10 
        xpoints = []
        ypoints = []
        """Give a list of waypoints, gives back entire path"""
        try: 
            print(request)
            print(request.points)
            pointsInput = request.points
            for point in range(len(request.points)):
                x = request.points[point].point.x
                y = request.points[point].point.y
                # points into lists of x and y inputs
                xpoints.append(x)
                ypoints.append(y)
            maxAccel = request.max_accel         

            # make a spline for the path
            path = CubicSpline(xpoints, ypoints)

            currentMaxSpeed = 0
            print(xpoints)
            for xval in np.arange(xpoints[0], xpoints[len(xpoints) - 1], 0.01):
                yval = float(path(xval))
                # If the xpoint is on the list, make it a point, immigrate data.
                if (xpoints.count(xval) != 0):
                    pointname = pointsInput[xpoints.index(xval)].point_name
                    # Create a point, if no name input, name = empty string so no errors
                    newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                        heading= pointsInput[xpoints.index(xval)].heading, 
                                        max_vel= pointsInput[xpoints.index(xval)].max_vel, 
                                        point_name= pointname if pointname is not None else "")
                    currentMaxSpeed = pointsInput[xpoints.index(xval)].max_vel
                else:
                    # If not on list, make a point
                    newPoint = Waypoint(point=Vector3(x=xval, y=yval, z=0.0), 
                                        heading= 0.0, # TODO Add heading enforcement
                                        max_vel= currentMaxSpeed, 
                                        point_name= "")
                pointsOutput.append(newPoint)
            # Connect created list of points to output
            response.waypoints = pointsOutput

            return response
        except Exception as e:
            print(e) 
            print("i did a dumb")
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
