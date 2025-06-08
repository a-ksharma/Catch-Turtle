import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
import random


class Spawn_Turtle(Node):
    def __init__(self):
        super().__init__("spawn_turtle")
        self.spawn = self.create_client(Spawn, '/spawn')
        while not self.spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again')
        self.timer_= self.create_timer(0.1, self.turtle_spawn)
        self.x = random.uniform(0.0, 10.0)
        self.y = random.uniform(0.0, 10.0)
        self.theta = random.choice([0.0, 1.57, 3.14, -1.57])

    def turtle_spawn(self):
        request = Spawn.Request()
        request.x = self.x
        request.y = self.y
        request.theta = self.theta
        request.name = "Target"
        future = self.spawn.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Turtle is Spawned Succesfully.")
        else:
            self.get_logger().info("Turtle is not Spawned")


def main():
    rclpy.init()
    node = Spawn_Turtle()
    node.turtle_spawn()
    node.destroy_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

