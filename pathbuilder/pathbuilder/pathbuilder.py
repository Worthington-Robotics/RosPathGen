from typing import List
import rclpy
from rclpy.node import Node
from rospathmsgs.srv import BakePath
from rospathmsgs.msg import Waypoint
from geometry_msgs.msg import Vector3

DEBUG = True

class pathBuilder(Node):
    def __init__(self):
        super().__init__('pathbuilder')
        self.pathBaker = self.create_client(BakePath, 'bake_path')
        self.declare_parameter('path_names', [""])
        
        pathNames = self.get_parameter("path_names").value

        for name in pathNames:
            path = self.declare_parameter(name + ".path", [""]).value
            self.future = self.send_request(name, path)
            
            
    def send_request(self, name: str, points: List[str]):
        request = BakePath.Request()
        request.max_velocity = self.declare_parameter(name + ".max_vel", 0.0).value
        request.max_accel = self.declare_parameter(name + ".max_accel", 0.0).value
        request.max_angular_vel = self.declare_parameter(name + ".max_ang_vel", 0.0).value
        request.path_name = name
        fixedPoints = []
        for strpoint in points:
            strpoint.replace(" ", "")
            self.get_logger().debug(strpoint)
            x = float(strpoint[(strpoint.find("x")+2): strpoint.find(",", (strpoint.find("x=")+1)-1)])
            y = float(strpoint[(strpoint.find("y")+2): strpoint.find(",", (strpoint.find("y=")+1)-1)])
            z = float(strpoint[(strpoint.find("z")+2): strpoint.find(")", (strpoint.find("z=")+1)-1)])
            vector = Vector3(x=x, y=y, z=z)
            head = float(strpoint[(strpoint.find("heading")+8): strpoint.find(",", (strpoint.find("heading")+1))])
            vel = float(strpoint[(strpoint.find("velocity")+9): strpoint.find(",", (strpoint.find("velocity")+1))])
            pointname = strpoint[(strpoint.find("point_name")+12): strpoint.find("'", (strpoint.find("point_name")+1))]
            waypoint = Waypoint(point=vector, heading=head, velocity=vel, point_name=pointname)
            self.get_logger().debug(waypoint)
            fixedPoints.append(waypoint)
        self.get_logger().debug(fixedPoints)
        request.points = fixedPoints
        return self.pathBaker.call_async(request)
    


def main():
    rclpy.init()
    node = pathBuilder()
    while rclpy.ok():
        rclpy.spin_once(node)
        
        if node.future.done():
            try:
                response = node.future.result()
            except Exception as e:
                node.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.success:
                    node.get_logger().info(f"Paths Sucessfully Built")
                else:
                    node.get_logger().error(response.message)
            break
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()