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
import math


def linearSpline(x, a, b):
    return a*x + b

def cubicSpline(x, a, b, c, d):
   return a * x^3 + b*x^2 + c*x + d


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
        timeParameterizedPoints = {} # Dictionary of Point: Time
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
                slope = (yvalues[1]-yvalues[0]/xvalues[1]-xvalues[0])
                #print(slope)
                constants = slope, (yvalues[0]-(xvalues[0]*slope))
                #print(yvalues[0]-(xvalues[0]*slope))
                linear = True
            else:
                constants, _ = curve_fit(cubicSpline, xvalues, yvalues)
                a,b,c,d = constants

            # Every velocity is it's own max velocity, 
            # basically slope at the max acceleration to the given velocity, 
            # then hold until either next velocity is 0 or max velocity changes            

                   

            # go through a loop where each point is at 0.01 seconds apart
            # nextxvalue = previousxvalue + previousVelocity*tdelta + maxAccel*tdelta^2
            
            tDelta = 0.01 # Constant for time between points in final trajectory
            currentTime = 0.0 # Current Time Var

            # Timestep is currently 0.01 seconds

            # Parameterize starting point
            
            previousVelocity = pointsInput[0].velocity
            timeParameterizedPoints[pointsInput[0]] = currentTime
            currentTime = currentTime + 0.01
            segmentAccel = maxAccel
            segmentAngVel = maxAngularVelocity
            
            for waypoint in pointsInput: # for every point in the list of input points
                if pointsInput.index(waypoint) == 0: continue
                prevWaypoint = pointsInput[pointsInput.index(waypoint)-1]
                startingHeading = prevWaypoint.heading
                segStartTime = timeParameterizedPoints[prevWaypoint]
                # If segment is supposed to be deccelerating, negate the acceleration
                if waypoint.velocity < prevWaypoint.velocity: 
                    segmentAccel = -maxAccel
                else:
                    segmentAccel = maxAccel
                # If heading is rotating left (negative) then negate max angular velocity
                segmentAngVel = -maxAngularVelocity if abs(prevWaypoint.heading - waypoint.heading) > 180 else maxAngularVelocity

                while nextPoint.point.x < waypoint.point.x:
                    #keep making waypoints every 0,01 seconds until done 
                    xval = prevWaypoint.point.x + prevWaypoint.velocity*tDelta + maxAccel*tDelta^2 # x val calculated from tdelta
                    yval = (float(cubicSpline(xval, *constants)) if not linear else float(linearSpline(xval, *constants))) # y val calculated from curve fitted earlier 
                    velocityVal = math.sqrt(previousVelocity**2 + 2*segmentAccel*(self.findDistance(prevWaypoint, xval, yval)))
                    headingVal = prevWaypoint.heading + (segmentAngVel * (currentTime-segStartTime)) # Heading Calculation using angular velocity
                    if headingVal < 0: headingVal = 360+headingVal # make sure negative headings don't happen
                    if segmentAngVel < 0: headingVal = (headingVal if headingVal < waypoint.heading else waypoint.heading) # Turning Left Overshoot Check
                    else: headingVal = (headingVal if headingVal > waypoint.heading else waypoint.heading) # Turning Right Overshoot Check
                    nextPoint = Waypoint(point=Vector3(x= xval, y=yval, z=0.0), 
                                         heading= headingVal, # 
                                         velocity= (velocityVal if velocityVal < maxVelocity else maxVelocity),
                                         point_name= "")
                    previousVelocity = velocityVal
                    timeParameterizedPoints[nextPoint] = currentTime
                    currentTime = currentTime + 0.01
                    prevWaypoint = nextPoint
               
                # Parameterize last point (also end waypoint)
                timeParameterizedPoints[waypoint] = currentTime
                currentTime = currentTime + 0.01
                
                
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
            for point in timeParameterizedPoints:
                pointsOutput.append(point)    
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
