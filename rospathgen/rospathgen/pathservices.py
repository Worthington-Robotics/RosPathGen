import rclpy
from rclpy.node import Node
from rospathmsgs.srv import BakePath, GetPath
from rospathgen.pathgenerator import PathGenerator
import os, sys

generator = PathGenerator()

class pathServices(Node):
    nameToPath = {}
    def bakePathCallback(self, request, response):
        print("I've got the path, am working on it, please don't stop me now.")
        name = request.path_name
        path = generator.generatePath(request)
        if len(path) == len(request.path):
            response.success = False
            response.message = "Your path could not successfully be generated please contact Tyler and check the output previous to this message."
            return response
        self.nameToPath[name] = path
        response.success = True
        print("Successfully obtained path")
        return response
        
    def getPathCallback(self, request, response):
        name = request.path_name
        try: 
            path = self.nameToPath[name]
            response.path = path
            print("Sucessfully found path with name {}".format(name))
        except Exception as e:
            print(e)
            print("Did not find path with name {}".format(name))
            print("Did you remember to create said path?")
            response = []
        return response

    def __init__(self):
        super().__init__('pathGen')
        self.bakepathsrv = self.create_service(BakePath, 'bake_path', self.bakePathCallback)
        self.getpathsrv = self.create_service(GetPath, 'get_path', self.getPathCallback)
        
# Run a node, don't try to understand
def main(args=None):
    rclpy.init(args=args)
    pathgen = pathServices()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
