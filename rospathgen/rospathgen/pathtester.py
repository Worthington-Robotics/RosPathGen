import rclpy
from rclpy.node import Node
from rospathmsgs.srv import GeneratePath
import numpy as np
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt

class tester(Node):

    def __init__(self):
        super().__init__('path_tester')
        self.client = self.create_client(GeneratePath, 'generate_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = GeneratePath.Request()

    def send_request(self):
        # Add point to list in format 
        # Waypoint(Vector3(x, y, z), heading, max_vel, point_name)
        self.request.points = [ Waypoint(point=Vector3(x=10.0, y=0.0, z=0.0), 
                                    heading=0.0, 
                                    max_vel=10.0, 
                                    point_name="Start"),
                                Waypoint(point=Vector3(x=5.0, y=1.0, z=0.0), 
                                    heading=0.0, 
                                    max_vel=10.0, 
                                    point_name="End")]
        self.request.max_accel = 20.0
        self.request.max_angular_vel = 20.0
        self.future = self.client.call_async(self.request)

def graphOutput(response):
    xvals = []
    yvals = []
    
    pointList = response.waypoints
    for point in pointList:
        xvals.append(point.point.x)
        yvals.append(point.point.y)
        if point.point_name != "":
            name = point.point_name
            name = str(name)
            #plt.text(xvals, yvals, name)        

    #print(xvals, yvals)
    plt.plot(xvals,yvals, 'bo')
    plt.xlabel('X value (m)')
    plt.ylabel('Y value (m)')
    plt.show()

def main():
    rclpy.init()

    pathTester = tester()
    pathTester.send_request()

    while rclpy.ok():
        rclpy.spin_once(pathTester)
        if pathTester.future.done():
            try:
                response = pathTester.future.result()
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
