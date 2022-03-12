import rclpy
from rclpy.node import Node
from rospathmsgs.srv import BakePath, GetPath, ListPaths
from rospathgen.pathgenerator import PathGenerator

generator = PathGenerator()

class pathServices(Node):
    nameToPath = {}
    def bakePathCallback(self, request, response):
        self.get_logger().info("I've got the path, am working on it, please don't stop me now.")
        name = request.path_name
        path = generator.generatePath(request)
        if len(path) == len(request.points) or len(request.points) == 0:
            response.success = False
            response.message = f"Your path with name: {name} could not successfully be generated please contact Tyler and check the output previous to this message."
            return response
        self.nameToPath[name] = path
        response.success = True
        response.message = name
        self.get_logger().info("Successfully obtained path")
        return response
        
    def getPathCallback(self, request, response):
        name = request.path_name
        try: 
            path = self.nameToPath[name]
            response.path = path
            self.get_logger().info("Sucessfully found path with name {}".format(name))
        except Exception as e:
            self.get_logger().fatal("Did not find path with name {}".format(name))
            self.get_logger().error("Did you remember to create said path?")
            response = []
        return response

    def listPathsCallback(self, request, response):
        paths = self.nameToPath.keys()
        response.names = paths
        return response

    def __init__(self):
        super().__init__('pathGen')
        self.bakepathsrv = self.create_service(BakePath, 'bake_path', self.bakePathCallback)
        self.getpathsrv = self.create_service(GetPath, 'get_path', self.getPathCallback)
        self.listpathssrv = self.create_service(ListPaths,'list_paths', self.listPathsCallback)
        
# Run a node, don't try to understand
def main(args=None):
    rclpy.init(args=args)
    pathgen = pathServices()
    rclpy.spin(pathgen)
    pathgen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
