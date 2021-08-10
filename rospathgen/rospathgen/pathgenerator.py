import rclpy
from rclpy import Node
from rospathmsgs.srv import GeneratePath

class pathGen(Node):

    def __init__(self):
        super().__init__('pathGen')


        



def main(args=None):
    rclpy.init(args=args)
    pathgen = pathGen()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
