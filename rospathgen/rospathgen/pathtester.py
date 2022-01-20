from cgi import test
import rclpy
from rclpy.node import Node
from rospathmsgs.srv import BakePath
from rospathmsgs.srv import GetPath
import numpy as np
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
import pylab
import matplotlib.pyplot as plt
import math

# Find Distance between 2 x and y values
def findDistance(startx, starty, endx, endy):
    return math.sqrt((endx - startx)**2 + (endy - starty)**2)


debug = 1 # 1 is nothing but essentials, 2 is somewhat verbose, 3 is everything
printPoints = True
printVelocities = True
printHeadings = True
graph = True

arbitraryName = "Haha this name is arbitrary meaning I can make it as long as I want muahahahahaha"
path = []

testingPath = [Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
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

tenFeetPath = [ Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=5.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=3.048, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="10 feet")]

philsTestPath = [ Waypoint(point=Vector3(x=0.0, y=0.0, z=0.0), 
                 heading=0.0, 
                 velocity=5.0, 
                 point_name="Start"),
                Waypoint(point=Vector3(x=0.5, y=0.5, z=0.0), 
                 heading=0.0, 
                 velocity=3.5, 
                 point_name="halfway"),
                Waypoint(point=Vector3(x=1.0, y=1.0, z=0.0), 
                 heading=0.0, 
                 velocity=0.0, 
                 point_name="1 meter")]





# Generic Template for a Waypoint
Waypoint(point=Vector3(x=9.0, y=5.0, z=0.0), 
                 heading=0.0, 
                 velocity=5.0, 
                 point_name="mid2")


# CHANGE THIS TO CHANGE YOUR PATH
path = philsTestPath

if path == testingPath: arbitraryName = "Testing Path"
elif path == tenFeetPath: arbitraryName = "10 Feet Path"
elif path == philsTestPath: arbitraryName = "Phil's Test Path"


class tester(Node):
    
    def __init__(self):
        super().__init__('path_tester')
        self.bakeclient = self.create_client(BakePath, 'bake_path')
        self.getclient = self.create_client(GetPath, 'get_path')
        self.initrequest = BakePath.Request()
        self.getrequest = GetPath.Request()

    def sendInitialRequest(self):
        # Add point to list in format 
        # Waypoint(Vector3(x, y, z), heading, velocity, point_name)
        self.initrequest.points = path
        self.initrequest.path_name = arbitraryName
        self.initrequest.max_velocity = 5.5 # m/s
        self.initrequest.max_accel = 4.0 # m/s/s
        self.initrequest.max_angular_vel = 90.0 # deg / s 
        self.initialFuture = self.bakeclient.call_async(self.initrequest)

    def sendGetRequest(self):
        self.getrequest.path_name = arbitraryName
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
        if debug >= 2: print("X: {}, Y: {}, Velocity: {}".format(point.point.x, point.point.y, point.velocity))
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
            if debug >= 2: print("Distance between points {}, {} and {}, {} is {}, which is not correct".format(prevxvalue, prevyvalue, xvalue, yvalue, distance))
    
    plt.figure()
    if printPoints:
        plt.subplot(221)
        plt.plot(xvals,yvals, 'bo')
        plt.xlabel('X value (m)')
        plt.ylabel('Y value (m)')
        for point in names.keys():
            x,y = point
            plt.text(x+0.6, y-0.3, names[point])
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

    if debug >= 1:
        print("Starting Wrong Statistics")
        print(" ")
        print("Max: {}".format(max(wrongDistances)))
        print("Min: {}".format(min(wrongDistances)))
        print("Std Dev: {}".format(np.std(wrongDistances)))
        print("Average: {}".format(np.average(wrongDistances)))

    if graph:
        fig = pylab.gcf()
        fig.canvas.manager.set_window_title(arbitraryName)
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
                if response.success: print("Path successfully completed")
                else: print("Path unsuccessfully completed with message {}".format(response.message))
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