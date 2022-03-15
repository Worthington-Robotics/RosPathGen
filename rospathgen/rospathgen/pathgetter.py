# make a node that when ran, gets a path of a name defined as a constant
# that then graphs the path using the same method as defined in the pathtester.py
# file.
from rclpy.node import Node
from rospathmsgs.srv import GetPath
import rclpy, pylab, math, sys
import numpy as np
import matplotlib.pyplot as plt

DEBUG = 1

printPoints = True
printVelocities = True
printHeadings = True
graph = True

class getter(Node):
    def __init__(self):
        super().__init__('path_getter')
        self.getclient = self.create_client(srv_type=GetPath, srv_name='/get_path')
        self.getrequest = GetPath.Request()

    def sendGetRequest(self):
        name = ""
        if sys.argv[1] is str: name = sys.argv[1]
        else: name = str(sys.argv[1])
        self.getrequest.path_name = name
        self.secondFuture = self.getclient.call_async(self.getrequest)

# Find Distance between 2 x and y values
def findDistance(startx, starty, endx, endy):
    return math.sqrt((endx - startx)**2 + (endy - starty)**2)


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
    if DEBUG >= 3: print(pointList)

    currentTime = 0
    for point in pointList:
        xvals.append(point.point.x)
        yvals.append(point.point.y)
        headings.append(point.heading)
        velocities.append(point.velocity)
        time.append(currentTime)
        if point.velocity == 0 and DEBUG >= 3:
            print(point.point.x, point.point.y)
        currentTime += 1
        if point.point_name != "":
            name = point.point_name
            name = str(name)
            names[(point.point.x, point.point.y)] = name       
        if DEBUG >= 2: print(f"X: {point.point.x}, Y: {point.point.y}, Velocity: {point.velocity}")
    
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
            if DEBUG >= 2: print(f"Distance between points {prevxvalue}, {prevyvalue} and {xvalue}, {yvalue} is {distance}, which is not correct")
    
    plt.figure()
    if printPoints:
        plt.subplot(221)
        plt.plot(xvals,yvals, 'bo')
        plt.xlabel('X value (m)')
        plt.ylabel('Y value (m)')
        plt.gca().set_aspect('equal', adjustable='datalim')
        plt.gca().autoscale()
        plt.gca().grid()
        # for point in names.keys():
        #     x,y = point
        #     plt.text(x+0.6, y-0.3, names[point])
    if printVelocities:
        plt.subplot(223)
        plt.plot(time,velocities, 'bo')
        plt.xlabel('Time')
        plt.ylabel('Velocity (m/s)')
        plt.gca().grid()
    if printHeadings:
        plt.subplot(222)
        plt.ylim([0, 360])
        plt.plot(time,headings, 'bo')
        plt.xlabel('Time')
        plt.ylabel('Heading (deg)')
        plt.gca().grid()
        # for point in path:
        #     if DEBUG >= 0: print(originalTimes)
        #     specificTime = originalTimes[path.index(point)]
        #     plt.plot(specificTime, point.heading, marker="o", markersize= 7, markeredgecolor="red", markerfacecolor="red")

    if DEBUG >= 1 and len(wrongDistances) != 0:
        print("Starting Wrong Statistics")
        print(" ")
        print("Max: {}".format(max(wrongDistances)))
        print("Min: {}".format(min(wrongDistances)))
        print("Std Dev: {}".format(np.std(wrongDistances)))
        print("Average: {}".format(np.average(wrongDistances)))

    if graph:
        fig = pylab.gcf()
        fig.canvas.manager.set_window_title(sys.argv[1])
        plt.show()


def main():
    rclpy.init()
    pathTester = getter()
    pathTester.sendGetRequest()
    while rclpy.ok():
        rclpy.spin_once(pathTester)       
        if pathTester.secondFuture.done():
            try:
                response = pathTester.secondFuture.result()
            except Exception as e:
                pathTester.get_logger().error(
                    'Service call failed %r' % (e,))
            else:
                graphOutput(response)
            break
    pathTester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()