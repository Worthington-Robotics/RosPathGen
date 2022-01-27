from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
from scipy.interpolate import splprep, splev
import numpy as np
import sys, os, math, time


debug = False

# Find Distance between 2 x and y values
def findDistance(startx, starty, endx, endy):
    return math.sqrt((endx - startx)**2 + (endy - starty)**2)

# Using a previous x value and a goal distance, finds next x value using slope and euclidian distance
def nextUValue(prevU, goalDist, constants=(0,0,0,0)):
    prevx, prevy = splev(prevU, constants)
    nextU = prevU
    nextX = prevx
    nextY = prevy
    distance = findDistance(prevx, prevy, nextX, nextY) # Find distance between the two points
    startTime = time.time() # Create timeout limit
    while distance < goalDist and (time.time()-startTime) < 1:
        nextU += 0.000025 # Increment u, any value lower than 0.000025 fails to have appreciable effect on distance 
                          # without running into more time constraints
        nextX, nextY = splev(nextU, constants) # Evaluate the next u value with respect to the function
        distance = findDistance(prevx, prevy, nextX, nextY) # Reevaluate distance between the two points
    return nextU

class PathGenerator(): 

    def generatePath(self, request):
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
        waypointIndexes = [] # List of all og waypoint indexes in pointsNoHead
        headingSlopes = {} # Diction in the form finalIndex, slope 

        """Give a list of waypoints, gives back entire path"""
        try: 
            pathStartTime = time.time()
            if debug: print(request)
            if debug: print(request.points)
            pointsInput = request.points
            for index in range(len(pointsInput)):
                x = pointsInput[index].point.x
                y = pointsInput[index].point.y
                # points into lists of x and y inputs
                xvalues.append(x)
                yvalues.append(y)
            # Pull Constants from request
            maxAccel = request.max_accel   
            maxVelocity = request.max_velocity
            maxAngularVelocity = request.max_angular_vel  

            # Scipy implementation of Curve Fitting
            if len(pointsInput) < 3:
                # Linear Paths
                constants, uGiven = splprep([xvalues, yvalues], k=1)
            else:
                constants = 0,0
                # Get Length to SciPy Minimum of 4
                if len(xvalues) < 4:
                    xvalue = xvalues[0]+xvalues[1]/2
                    yvalue = yvalues[0]+yvalues[1]/2
                    xvalues.insert(1, xvalue)
                    yvalues.insert(1, yvalue)
                if debug: print(xvalues, yvalues) 
                constants, uGiven = splprep([xvalues, yvalues])
                uGiven = np.delete(uGiven, 1)
            
            # Stage 1: Get all the points, no headings, no velocities
            for point in pointsInput:
                if pointsInput.index(point) == 0: 
                    pointsNoHeadNoVel.append(point)
                    continue
                prevPoint = pointsInput[pointsInput.index(point)-1]
                prevIndex = pointsInput.index(point)-1
                prevU = uGiven[prevIndex]
                finalU = uGiven[pointsInput.index(point)]
                # go through a loop where each point is at 0.01 meters of distance (x and y) !! apart
                # point is starting point find next x value by integral(a, b, sqrt(1+(f'(x))^2))dx = distance b/w points
                nextU = nextUValue(prevU, 0.01, constants)
                startTime = time.time() # create timeout condition
                while nextU < finalU and (time.time()-startTime) < 1:
                    # Make a point at (nextX, yval)
                    xval, yval = splev(nextU, constants) 
                    # y val calculated from curve fitted earlier 
                    if debug: print("xval: {} yval: {}".format(xval, yval))
                    nextPoint = Waypoint(point=Vector3(x= float(xval), y=float(yval), z=0.0), 
                                        heading= 0.0,  
                                        velocity= 0.0,
                                        point_name= "")
                    pointsNoHeadNoVel.append(nextPoint)
                    # Find the next x value
                    prevU = nextU
                    nextU = nextUValue(prevU, 0.01, constants)
                pointsNoHeadNoVel.append(point)
            
            
            # Stage 2: Velocity
            # Forward Pass
            
            currentMaxVel = maxVelocity

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
                    if debug: print("Velocity Updated to: {} at point ({},{})".format(currentMaxVel, point.point.x, point.point.y))
                prevPoint = pointsNoHeadFWVel[pointsNoHeadFWVel.index(point)+1]
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
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
                leftDist = prevPoint.heading - (point.heading-360)
                rightDist = point.heading - prevPoint.heading
                if point.heading == prevPoint.heading:
                    # No Heading Change over interval
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = 0
                elif leftDist > rightDist:
                    # Right is shorter, go right
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = ((point.heading-prevPoint.heading)/(pointIndex-prevPointIndex))
                else:
                    # Left is shorter, go left
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = ((point.heading-360)-prevPoint.heading)/(pointIndex-prevPointIndex)
                headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)]] = 0


            segmentAngVel = 0
            prevHeading = 0
            # max angular velocity support
            for AngVel in headingSlopes:
                if AngVel > maxAngularVelocity: AngVel = maxAngularVelocity
            if debug: print(waypointIndexes)
            if debug: print(headingSlopes)
            for point in pointsNoHead:
                index = pointsNoHead.index(point)
                if index in waypointIndexes:
                    segmentAngVel = headingSlopes[index] # update the segment angular velocity if the segment angular velocity exists
                    pointsOutput.append(point)
                    continue
                headingVal = prevHeading + segmentAngVel # Heading Calculation using angular velocity
                if headingVal < 0: headingVal = 360+headingVal # make sure negative headings don't happen
                point.heading = float(headingVal)
                pointsOutput.append(point)
                prevHeading = headingVal      
            print("That path took {} seconds to complete.".format(time.time()-pathStartTime))
            return pointsOutput
        except Exception as e:
            print(e) 
            print("i did a dumb")
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            print(exc_type, fname, exc_tb.tb_lineno)
            return pointsOutput
        