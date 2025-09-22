import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, TeleportAbsolute

class Spawner(Node):
    def __init__(self):
        super().__init__('turtle_pong_spawner')
        self.cli = self.create_client(Spawn, 'spawn')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('waiting for /spawn service...')


# spawn paddles and ball
        self.spawn_turtle(2.0, 5.5, 0.0, 'paddle_left')
        self.spawn_turtle(9.0, 5.5, 0.0, 'paddle_right')
        self.spawn_turtle(5.5, 5.5, 0.0, 'ball')
        self.get_logger().info('spawned turtles: paddle_left, paddle_right, ball')


    def spawn_turtle(self, x, y, theta, name):
        req = Spawn.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        req.name = name
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        if fut.result() is None:
            self.get_logger().error('Failed to call spawn service')
        else:
            self.get_logger().info(f"Spawned '{name}' at {x},{y}")




def main(args=None):
    rclpy.init(args=args)
    node = Spawner()
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()