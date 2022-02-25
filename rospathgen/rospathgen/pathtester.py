import rclpy, pylab, math
from rclpy.node import Node
from rospathmsgs.srv import BakePath, GetPath
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
import numpy as np
import matplotlib.pyplot as plt
import random

# Find Distance between 2 x and y values
def findDistance(startx, starty, endx, endy):
    return math.sqrt((endx - startx)**2 + (endy - starty)**2)


debug = 1 # 1 is nothing but essentials, 2 is somewhat verbose, 3 is everything
printPoints = True
printVelocities = True
printHeadings = True
graph = True

pathName = "Haha this name is arbitrary meaning I can make it as long as I want muahahahahaha"
path = []

testingPath = [
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                        heading=0.0, 
                        velocity=5.0, 
                        point_name="Start"),
                Waypoint(point=Vector3(x=3.0, y=7.0, z=0.0), 
                        heading=0.0, 
                        velocity=3.5,
                        point_name=""),
                Waypoint(point=Vector3(x=5.0, y=3.0, z=0.0), 
                        heading=90.0, 
                        velocity=5.0, 
                        point_name=""),
                Waypoint(point=Vector3(x=4.0, y=4.5, z=0.0), 
                        heading=300.0, 
                        velocity=0.0, 
                        point_name="End")]

tenFeetPath = [ 
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=1.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=4.0, y=1.0, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="bend"),
                 Waypoint(point=Vector3(x=3.0, y=-2.0, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="bendy")
]

testCurvy = [ 
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=1.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=3.048, y=0.6096, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="10 ft, 2 ft")]

philsTestPath = [ 
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=5.0, y=1.0, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="2"),
                Waypoint(point=Vector3(x=3.0, y=-2.0, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="3"),
                Waypoint(point=Vector3(x=1.0, y=-3.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="End")]


sixCentimeters = [
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.5, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=0.06, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="6 cm")]


autoOneBallPickUp = [ 
                Waypoint(point=Vector3(x=7.44, y=-2.34, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=7.90, y=-3.81, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                ]
autoOneShootOne = [
                Waypoint(point=Vector3(x=7.90, y=-3.81, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                Waypoint(point=Vector3(x=5.69, y=-3.81, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting One Location"),
                ]
autoOneShootTwo = [
                Waypoint(point=Vector3(x=5.69, y=-3.81, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting One Location"),
                Waypoint(point=Vector3(x=4.50, y=-2.79, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting Two Location"),
                ]
                


autoThreeBallPickUp = [
                Waypoint(point=Vector3(x=6.68, y=-1.63, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=5.17, y=-2.44, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                ]
autoThreeShootOne = [
                Waypoint(point=Vector3(x=6.00, y=-2.50, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                Waypoint(point=Vector3(x=5.17, y=-2.44, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting One Location"),
                ]
autoThreeShootTwo = [
                Waypoint(point=Vector3(x=5.17, y=-2.44, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting One Location"),
                Waypoint(point=Vector3(x=4.09, y=-1.07, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting Two Location"),
                ]



autoFourBallPath = [
                Waypoint(point=Vector3(x=6.10, y=0.10, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=4.45, y=-1.07, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Hitting Location"),
                ]



autoTwoBallPickUp = [
                Waypoint(point=Vector3(x=6.53, y=1.22, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=4.88, y=2.01, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                ]
autoTwoShootOne = [
                Waypoint(point=Vector3(x=4.88, y=2.01, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Pick Up Ball Location"),
                Waypoint(point=Vector3(x=3.66, y=-1.35, z=0.0), 
                 heading=0.0, 
                 velocity=2.0, 
                 point_name="Shooting One Location"),
                ]




threePointCurvy = [ 
                Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=5.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=0.5, y=0.5, z=0.0), 
                 heading=0.0, 
                 velocity=3.5, 
                 point_name="halfway"),
                Waypoint(point=Vector3(x=0.0, y=1.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="1 meter")]


                 

# Generic Template for a Waypoint
Waypoint(point=Vector3(x=9.0, y=5.0, z=0.0), 
                 heading=0.0, 
                 velocity=5.0, 
                 point_name="mid2")





# CHANGE THIS TO CHANGE YOUR PATH
path =  autoTwoShootOne
max_velocity = 5.5 # m/s
max_accel = 5.0 # m/s/s
max_angular_vel = 90.0 # deg / s +

index = 0

# Set Names Specifically when baking
if path == testingPath: pathName = "Testing Path"
elif path == tenFeetPath: pathName = "six"
elif path == philsTestPath: pathName = "Phil's Test Path"
elif path == sixCentimeters: pathName = "six"
elif path == autoOneBallPickUp: pathName = "Auto 1: Ball Pick Up"
elif path == autoOneShootOne: pathName = "Auto 1: First Shot"
elif path == autoOneShootTwo: pathName = "Auto 1: Second Shot"
elif path == autoTwoBallPickUp: pathName = "Auto 2: Ball Pick Up"
elif path == autoTwoShootOne : pathName = "Auto 2: First Shot"
elif path == autoTwoShootTwo: pathName = "Auto 2: Second Shot"

else: pathName = str(random.random())



class tester(Node):
    
    def __init__(self):
        super().__init__('path_tester')
        self.bakeclient = self.create_client(BakePath, '/bake_path')
        self.getclient = self.create_client(GetPath, '/get_path')
        self.initrequest = BakePath.Request()
        self.getrequest = GetPath.Request()

    def sendInitialRequest(self):
        # Add point to list in format 
        # Waypoint(Vector3(x, y, z), heading, velocity, point_name)
        self.initrequest.points = path
        self.initrequest.path_name = pathName
        self.initrequest.max_velocity = max_velocity # m/s
        self.initrequest.max_accel = max_accel # m/s/s
        self.initrequest.max_angular_vel = 90.0 # deg / s 
        self.initialFuture = self.bakeclient.call_async(self.initrequest)

    def sendGetRequest(self):
        self.getrequest.path_name = pathName
        self.secondFuture = self.getclient.call_async(self.getrequest)


def graphOutput(response):
    xvals = []
    yvals = []
    headings = []
    velocities = []
    time = []
    originalTimes = []
    wrongDistances = []
    names = {} # of point(tuple) to name

    originalTimes.append(0)

    pointList = response.path
    if debug >= 3: print(pointList)

    currentTime = 0
    for point in pointList:
        xvals.append(point.point.x)
        yvals.append(point.point.y)
        headings.append(point.heading)
        velocities.append(point.velocity)
        time.append(currentTime)
        if point.velocity == 0 and debug >= 3:
            print(point.point.x, point.point.y)
        currentTime += 1
        if point.point_name != "":
            name = point.point_name
            name = str(name)
            names[(point.point.x, point.point.y)] = name       
        if debug >= 2: print(f"X: {point.point.x}, Y: {point.point.y}, Velocity: {point.velocity}")
        if point in path: 
            originalTimes.append(currentTime)
    
    for xvalue in xvals:
        index = xvals.index(xvalue)
        if index == 0: continue
        prevIndex = index - 1
        yvalue = yvals[index]
        prevxvalue = xvals[prevIndex]
        prevyvalue = yvals[prevIndex]
        distance = findDistance(prevxvalue, prevyvalue, xvalue, yvalue)
        if distance > 0.01 or distance < 0.01:
            wrongDistances.append(distance)
            if debug >= 2: print(f"Distance between points {prevxvalue}, {prevyvalue} and {xvalue}, {yvalue} is {distance}, which is not correct")
    
    plt.figure()
    if printPoints:
        plt.subplot(221)
        plt.plot(xvals,yvals, 'bo')
        plt.xlabel('X value (m)')
        plt.ylabel('Y value (m)')
        # for point in names.keys():
        #     x,y = point
        #     plt.text(x+0.6, y-0.3, names[point])
        for point in path:
            plt.plot(point.point.x, point.point.y, marker="o", markersize= 7, markeredgecolor="red", markerfacecolor="red")
    if printVelocities:
        plt.subplot(223)
        plt.plot(time,velocities, 'bo')
        plt.xlabel('Time')
        plt.ylabel('Velocity (m/s)')
    if printHeadings:
        plt.subplot(222)
        plt.ylim([0, 360])
        plt.plot(time,headings, 'bo')
        plt.xlabel('Time')
        plt.ylabel('Heading (deg)')
        # for point in path:
        #     if debug >= 0: print(originalTimes)
        #     specificTime = originalTimes[path.index(point)]
        #     plt.plot(specificTime, point.heading, marker="o", markersize= 7, markeredgecolor="red", markerfacecolor="red")

    if debug >= 1 and len(wrongDistances) != 0:
        print("Starting Wrong Statistics")
        print(" ")
        print("Max: {}".format(max(wrongDistances)))
        print("Min: {}".format(min(wrongDistances)))
        print("Std Dev: {}".format(np.std(wrongDistances)))
        print("Average: {}".format(np.average(wrongDistances)))

    if graph:
        fig = pylab.gcf()
        fig.canvas.manager.set_window_title(pathName)
        plt.show()


def main():
    rclpy.init()

    pathTester = tester()
    pathTester.sendInitialRequest()
    while rclpy.ok():
        rclpy.spin_once(pathTester)       
        if pathTester.initialFuture.done():
            try:
                response = pathTester.initialFuture.result()
            except Exception as e:
                pathTester.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.success: print(f"Path successfully completed with name: {pathName}")
                else: print(f"Path unsuccessfully completed with message {response.message}")
            break
    pathTester.sendGetRequest()
    while rclpy.ok():
        rclpy.spin_once(pathTester)       
        if pathTester.secondFuture.done():
            try:
                response = pathTester.secondFuture.result()
            except Exception as e:
                pathTester.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                graphOutput(response)
            break
    pathTester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
