from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
from scipy.interpolate import splprep, splev
import numpy as np
import sys, os, math, time
from rclpy.node import Node
from math import pi


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
        nextU += 0.000725 # Increment u, any value lower than 0.000025 fails to have appreciable effect on distance 
                          # without running into more time constraints
        nextX, nextY = splev(nextU, constants) # Evaluate the next u value with respect to the function
        distance = findDistance(prevx, prevy, nextX, nextY) # Reevaluate distance between the two points
    return nextU

class PathGenerator(): 

    def __init__(self, node: Node):
        self.logger = node.get_logger()

    def generatePath(self, request: Waypoint):
        pointsInput = [] # Input list of points
        pointsNoHeadNoVel = [] # Points without heading or velocity
        pointsNoHeadFWVel = [] # Points with no heading, no backwards velocity pass
        pointsNoHead = [] # Points with no heading
        pointsNoAngVel = [] #Points with heading, fw & backwards vel, no angular velocity
        pointsOutput = [] # Output list of points with heading and velocity
        maxAccel = 8 # Max Acceleration (Default)
        maxVelocity = 10 # Max Velocity (Default)
        timeLimit = 500 #Amount of iterations before the velocity assigner gives up
        # X and Y Seperated Input Values
        xvalues = []
        yvalues = []
        waypointIndexes = [] # List of all og waypoint indexes in pointsNoHead
        headingSlopes = {} # Diction in the form finalIndex, slope 

        """Give a list of waypoints, gives back entire path"""
        try: 
            pathStartTime = time.time()
            # if debug: print(request)
            self.logger.debug("Requested points {}".format(request.points))
            pointsInput = request.points
            for index in range(len(pointsInput)):
                x = pointsInput[index].point.x
                y = pointsInput[index].point.y

                pointsInput[index].heading = pointsInput[index].heading / 180.0 * pi # convert heading to radians

                # points into lists of x and y inputs
                xvalues.append(x)
                yvalues.append(y)
            # Pull Constants from request
            maxAccel = request.max_accel   
            maxVelocity = request.max_velocity
            maxAngularVelocity = request.max_angular_vel  

            # perform order selection
            if debug: print(xvalues, yvalues) 
            k = 3
            if len(xvalues) <= k:
                k = len(xvalues) - 1
                self.logger.warning("Could not keep 3rd order spline on path with {} points. Switching to order {}".format(len(xvalues), k))
                

            (constants, uGiven), fpGiven, ier, msg = splprep([xvalues, yvalues], s=0, full_output=1, k=k)
            if ier > 0:
                self.logger.error('error during splprep: {}'.format(msg))
                return

            self.logger.debug("len of uGiven {}".format(len(uGiven)))
            
            # Stage 1: Get all the points, no headings, no velocities
            startTime = time.time() # create timeout condition

            currentU = 1
            for u in np.linspace(0,1,200):
                if u == 0:
                    pointsNoHeadNoVel.append(pointsInput[0])
                    continue
                elif u == 1:
                    pointsNoHeadNoVel.append(pointsInput[-1])
                    continue
                if (u < uGiven[currentU]):
                    xval, yval = splev(u, constants)
                else:
                    # inject the point betweens
                    pointsNoHeadNoVel.append(pointsInput[currentU])

                    xval, yval = splev(uGiven[currentU], constants)
                    currentU = currentU + 1

                nextPoint = Waypoint(point = Vector3(x = float(xval), y = float(yval), z = 0.0), 
                                heading = 0.0,  
                                velocity = 0.0,
                                point_name = "")
                pointsNoHeadNoVel.append(nextPoint)

            # print("Stage 1 points {}".format(pointsNoHeadNoVel))

            self.logger.debug(f"Stage One Time: {time.time() - startTime}")
            startTime = time.time()    
            
            # Stage 2: Velocity
            # Forward Pass
            currentMaxVel = maxVelocity

            # DESIGN STANDARD: Any Start or End Point should have a velocity of 0
            #pointsInput[0].velocity = 0.0
            pointsInput[-1].velocity = 0.0
            
            # Turn no headings/velocity into forward pass w/ velocity
            timeElapsed = 0 
            for point in pointsNoHeadNoVel:
                if pointsNoHeadNoVel.index(point) == 0:
                    self.logger.debug(f"Reset Max Velocity to 0 at point ({point.point.x},{point.point.y})")
                        
                    currentMaxVel = point.velocity
                    point.velocity = 0.0
                    pointsNoHeadFWVel.append(point)
                    continue

                prevPoint = pointsNoHeadNoVel[pointsNoHeadNoVel.index(point) - 1]

                if point.velocity > 0:
                    currentMaxVel = point.velocity
                    #self.logger.debug(f"Velocity Updated to: {currentMaxVel} at point ({point.point.x},{point.point.y})")
                distance = abs(findDistance(point.point.x, point.point.y, prevPoint.point.x, prevPoint.point.y))
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
                vel = math.sqrt(prevPoint.velocity ** 2 + 2 * maxAccel * distance)
                # If less than max, use it, else, use max.
                point.velocity = vel if vel < currentMaxVel else currentMaxVel
                # If path is not feasible, abort
                timeDelta = (2 / (point.velocity + prevPoint.velocity)) * distance
                timeElapsed += timeDelta
                if timeElapsed > timeLimit:
                    self.logger.warning(f"Stage Two Time but path isn't acceptable:{time.time() - startTime}")
                    startTime = time.time()
                    return request.points
                pointsNoHeadFWVel.append(point)
            self.logger.debug(f"Stage Two Time: {time.time() - startTime}")
            startTime = time.time()

            # Stage 3: Backward Pass
            reversedPointsNoHeadFWVel = pointsNoHeadFWVel
            reversedPointsNoHeadFWVel.reverse()

            timeElapsed = 0 
            for point in reversedPointsNoHeadFWVel:
                currentIndex = reversedPointsNoHeadFWVel.index(point)
                if currentIndex == 0:
                    currentMaxVel = point.velocity
                    point.velocity = 0.0
                    pointsNoHead.insert(0, point)
                    continue
                if point.velocity > currentMaxVel:
                    currentMaxVel = point.velocity
                    self.logger.debug("Velocity Updated to: {} at point ({},{})".format(currentMaxVel, point.point.x, point.point.y))

                prevPoint = reversedPointsNoHeadFWVel[currentIndex-1]
                distance = abs(findDistance(point.point.x, point.point.y, prevPoint.point.x, prevPoint.point.y))
                # Enforce global max velocity and max reachable velocity by global acceleration limit.
                # vf = sqrt(vi^2 + 2*a*d)
                vel = math.sqrt(prevPoint.velocity ** 2 + 2 * maxAccel * distance)
                if vel > currentMaxVel: vel = currentMaxVel
                #self.logger.debug("Vel Value {} at Point {}, {} with previous point {}, {} vel value {}".format(vel, point.point.x, point.point.y, prevPoint.point.x, prevPoint.point.y, prevPoint.velocity))
                point.velocity = vel if point.velocity > vel else point.velocity
                # If path is not feasible, abort
                timeDelta = (2/(point.velocity + prevPoint.velocity))* distance
                timeElapsed += timeDelta
                if timeElapsed > timeLimit: 
                    self.logger.warning(f"Stage Three Time but the path isn't acceptable:{time.time() - startTime}")
                    startTime = time.time()
                    return request.points
                pointsNoHead.insert(0, point)
            self.logger.debug(f"Stage Three Time: {time.time() - startTime}")
            startTime = time.time()
            
            # Stage 4: Heading Enforcement
            # Go through and find the heading changes
            prevPoint = None
            for point in pointsInput:
                
                pointIndex = pointsNoHead.index(point)
                waypointIndexes.append(pointIndex)

                # Skip 1st Point
                if pointIndex == 0: 
                    prevPoint = point
                    continue

                prevPointIndex = pointsNoHead.index(prevPoint)

                # Find Left Distance, Find Right Distance, Figure out Which is shorter
                leftDist = prevPoint.heading - (point.heading-2*pi)
                rightDist = point.heading - prevPoint.heading

                if point.heading == prevPoint.heading:
                    # No Heading Change over interval
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = 0
                elif leftDist > rightDist:
                    # Right is shorter, go right
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = ((point.heading-prevPoint.heading)/(pointIndex-prevPointIndex))
                else:
                    # Left is shorter, go left
                    headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)-1]] = ((point.heading-2*pi)-prevPoint.heading)/(pointIndex-prevPointIndex)

                headingSlopes[waypointIndexes[waypointIndexes.index(pointIndex)]] = 0

                # set prev point
                prevPoint = point


            segmentAngVel = 0
            prevHeading = 0
            # max angular velocity support
            for AngVel in headingSlopes:
                if AngVel > maxAngularVelocity: AngVel = maxAngularVelocity

            self.logger.debug(f"Control point indicies: {waypointIndexes}")
            self.logger.debug(f"Heading slopes: {headingSlopes}")

            for point in pointsNoHead:

                index = pointsNoHead.index(point)

                # Skip 1st Point and take its heading as the predecesor heading
                if index == 0: 
                    prevHeading = point.heading
                    segmentAngVel = headingSlopes[0]
                    continue

                # adjust the rate at each key point
                if index in waypointIndexes:
                    segmentAngVel = headingSlopes[index] # update the segment angular velocity if the segment angular velocity exists
                    pointsOutput.append(point)
                    continue
                
                # Heading Calculation using angular velocity
                headingVal = prevHeading + segmentAngVel 

                # make sure negative headings don't happen
                if headingVal < 0: headingVal = 2*pi + headingVal 

                point.heading = float(headingVal)

                # self.logger.debug(f"evaluated point {index}: {point}")   

                pointsOutput.append(point)
                prevHeading = headingVal  
            
            # for i in range(len(pointsOutput)):
            #     self.logger.debug(f"resultant path point {i}: {pointsOutput[i]}")   

            self.logger.debug(f"Stage Four Time: {time.time() - startTime}")    
            self.logger.info(f"Path took {time.time() - pathStartTime} seconds to complete.")
            return pointsOutput
        except Exception as e:
            exc_type, exc_obj, exc_tb = sys.exc_info()
            fname = os.path.split(exc_tb.tb_frame.f_code.co_filename)[1]
            self.logger.error(f"Caught exception during generation {type(e)}: {e}, {exc_type, fname, exc_tb.tb_lineno}")
            self.logger.debug(f"{request.points}")
            return pointsOutput