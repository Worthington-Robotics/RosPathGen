from asyncio.format_helpers import _format_callback_source
from textwrap import fill
from turtle import heading
import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
from scipy.optimize import curve_fit
from scipy.interpolate import CubicHermiteSpline
from scipy.interpolate import interp1d
import sys
import os
import math
import time


debug = True

# Linear and Cubic Spline Functions for use with SciPy Later on
def linearSpline(x, a, b):
    return a*x + b



# Find Distance between a waypoint and a x and y value
def findDistance(startx, starty, endx, endy):
    return math.sqrt((endx - startx)**2 + (endy - starty)**2)

# Using a previous x value and a goal distance, finds next x value using slope and euclidian distance
def nextXPoint(prevx, prevy, goalDist, linear, curve, constants=(0,0,0,0)):
    nextX = prevx
    nextY = prevy
    distance = findDistance(prevx, prevy, nextX, nextY)
    startTime = time.time()
    while distance < goalDist and (time.time()-startTime) < 1:
        nextX += 0.01 # Increment x by 0.1 mm
        nextY = (float(curve(nextX)) if not linear else float(linearSpline(nextX, *constants)))
        distance = findDistance(prevx, prevy, nextX, nextY)
    #print(nextX)
    return nextX

class pathGen(Node): 
    def generatepathcallback(self, request, response):
        pointsInput = [] # Input list of points
        pointsNoHeadNoVel = [] # Points without heading or velocity
        pointsNoHeadFWVel = [] # Points with no heading, no backwards velocity pass
        pointsNoHead = [] # Points with no heading
        pointsOutput = [] # Output list of points with heading and velocity
        maxAccel = 8 # Max Acceleration (Default)
        maxVelocity = 10 # Max Velocity (Default)
        # X and Y Seperated Input Values
        xvalues = []
        yvalues = []
        derivatives = []
        pointsAsTuples = [] # List of the points as tuples
        waypointIndexes = [] # List of all og waypoint indexes in pointsNoHead
        headingSlopes = {} # Diction in the form finalIndex, slope 
        linear = False # is the path linear?
        """Give a list of waypoints, gives back entire path"""
        try: 
            #print(request)
            #print(request.points)
            pointsInput = request.points
            for waypoint in range(len(pointsInput)):
                x = pointsInput[waypoint].point.x
                y = pointsInput[waypoint].point.y
                # points into lists of x and y inputs
                xvalues.append(x)
                yvalues.append(y)
                pointsAsTuples.append((x,y))
            # Pull Constants from request
            maxAccel = request.max_accel   
            maxVelocity = request.max_velocity
            maxAngularVelocity = request.max_angular_vel  

            

            # Scipy implementation of Curve Fitting
            if len(pointsInput) < 3:
                slope = ((yvalues[1]-yvalues[0])/(xvalues[1]-xvalues[0]))
                if debug: print("Slope: {}".format(slope))
                # y-mx = b
                intercept = yvalues[0]-(slope*xvalues[0])
                constants = slope, intercept
                if debug: print("Intercept: {}".format(intercept))
                linear = True
                curve = 0
            else:
                constants = 0,0
                # Get Length to SciPy Minimum of 4
                if len(xvalues) < 4:
                    xvalue = xvalues[0]+xvalues[1]/2
                    yvalue = yvalues[0]+yvalues[1]/2
                    xvalues.insert(1, xvalue)
                    yvalues.insert(1, yvalue)
                
                # Find 1st Derivatives for each segment
                for xvalue in xvalues:
                    if xvalues.index(xvalue) == len(xvalues)-1:
                        derivatives.append(0)
                        break
                    index = xvalues.index(xvalue)
                    xvalue = xvalues[index]+xvalues[index+1]/2
                    yvalue = yvalues[index]+yvalues[index+1]/2
                    slope = yvalue/xvalue
                    derivatives.append(slope)
                print(xvalues, yvalues, derivatives)
                #curve = CubicHermiteSpline(xvalues, yvalues, derivatives)
                curve = interp1d(xvalues, yvalues, kind='cubic', fill_value="extrapolate")

            # Stage 1: Get all the points, no headings, no velocities

            for point in pointsInput:
                if pointsInput.index(point) == 0: 
                    pointsNoHeadNoVel.append(point)
                    continue
                prevPoint = pointsInput[pointsInput.index(point)-1]
                slope = (point.point.y-prevPoint.point.y)/(point.point.x-prevPoint.point.x)
                # go through a loop where each point is at 0.01 meters of distance (x and y) !! apart
                # point is starting point find next x value by integral(a, b, sqrt(1+(f'(x))^2))dx = distance b/w points
                nextX = nextXPoint(prevPoint.point.x, prevPoint.point.y, 0.01, linear, curve, constants)
                startTime = time.time()
                while nextX < point.point.x and (time.time()-startTime) < 1:
                    # Make a point at (nextX, yval)
                    yval = (float(curve(nextX)) if not linear else float(linearSpline(nextX, *constants))) # y val calculated from curve fitted earlier 
                    if debug: print("xval: {} yval: {}".format(nextX, yval))
                    nextPoint = Waypoint(point=Vector3(x= nextX, y=yval, z=0.0), 
                                        heading= 0.0,  
                                        velocity= 0.0,
                                        point_name= "")
                    pointsNoHeadNoVel.append(nextPoint)
                    # Find the next x value
                    nextX = nextXPoint(nextX, yval, 0.01, linear, curve, constants)
                pointsNoHeadNoVel.append(point)
            
            
            # Stage 2: Velocity
            # Forward Pass
            
            currentMaxVel = pointsInput[0].velocity

            # DESIGN STANDARD: Any Start or End Point should have a velocity of 0
            #pointsInput[0].velocity = 0.0
            pointsInput[len(pointsInput)-1].velocity = 0.0
            
            for point in pointsNoHeadNoVel:
                prevPoint = pointsNoHeadNoVel[pointsNoHeadNoVel.index(point)-1]
                if pointsNoHeadNoVel.index(point) == 0:
                    if debug: print("Reset Max Velocity to 0 at point ({},{})".format(point.point.x, point.point.y))
                    currentMaxVel = point.velocity
                    point.velocity = 0.0
                    pointsNoHeadFWVel.append(point)
                    continue
                if point.velocity > 0:
                    currentMaxVel = point.velocity
                    if debug: print("Velocity Updated to: {}at point ({},{})".format(currentMaxVel, point.point.x, point.point.y))
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
                vel = math.sqrt(prevPoint.velocity**2 + 2*maxAccel)
                point.velocity = vel if vel < currentMaxVel else currentMaxVel
                pointsNoHeadFWVel.append(point)

            # Stage 3: Backward Pass
            for point in reversed(pointsNoHeadFWVel):
                if pointsNoHeadFWVel.index(point) == len(pointsNoHeadFWVel)-1:
                    currentMaxVel = point.velocity
                    point.velocity = 0.0
                    pointsNoHead.insert(0, point)
                    continue
                if point.velocity > currentMaxVel:
                    currentMaxVel = point.velocity
                    if debug: print("Velocity Updated to: {}at point ({},{})".format(currentMaxVel, point.point.x, point.point.y))
                prevPoint = pointsNoHeadFWVel[pointsNoHeadFWVel.index(point)+1]
                vel = math.sqrt(prevPoint.velocity**2 + 2*maxAccel)
                if vel > currentMaxVel: vel = currentMaxVel
                if debug: print("Vel Value {} at Point {}, {} with previous point {}, {}".format(vel, point.point.x, point.point.y, prevPoint.point.x, prevPoint.point.y))
                point.velocity = vel if point.velocity > vel else point.velocity
                pointsNoHead.insert(0, point)

            # DESIGN STANDARD: Any Start Point should have a heading of 0
            pointsNoHead[0].heading = 0.0
            
            # Stage 4: Heading Enforcement
            # Go through and find the heading changes
            for point in pointsInput:
                # Skip 1st Point
                if pointsInput.index(point) == 0: 
                    pointIndex = pointsNoHead.index(point)
                    waypointIndexes.append(pointIndex)
                    continue
                prevPoint = pointsInput[pointsInput.index(point)-1]
                pointIndex = pointsNoHead.index(point)
                prevPointIndex = pointsNoHead.index(prevPoint)
                waypointIndexes.append(pointIndex)
                # Find Left Distance, Find Right Distance, Figure out Which is shorter
                leftDist = point.heading+360 - prevPoint.heading
                rightDist = point.heading - prevPoint.heading
                if point.heading == prevPoint.heading:
                    # No Heading Change over interval
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = 0
                elif leftDist > rightDist:
                    # Right is shorter, go right
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = ((point.heading-prevPoint.heading)/(pointIndex-prevPointIndex))
                else:
                    # Left is shorter, go left
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = -((point.heading-prevPoint.heading)/(pointIndex-prevPointIndex))
                headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)]] = 0


            segmentAngVel = 0
            prevHeading = 0
            for point in pointsNoHead:
                index = pointsNoHead.index(point)
                if index in waypointIndexes:
                    print(index)
                    print(waypointIndexes)
                    print(headingSlopes)
                    segmentAngVel = headingSlopes[index]
                    pointsOutput.append(point)
                    continue

                headingVal = prevHeading + segmentAngVel # Heading Calculation using angular velocity
                if headingVal < 0: headingVal = 360+headingVal # make sure negative headings don't happen
                point.heading = float(headingVal)
                pointsOutput.append(point)
                prevHeading = headingVal               
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
