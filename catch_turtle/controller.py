import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
import random
import math

class Control_Turtle(Node):
    def __init__(self):
        super().__init__("control_turtle")
        self.spawn = self.create_client(Spawn, '/spawn')
        while not self.spawn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn service not available, waiting...')
        self.pose_subscriber_ = self.create_subscription(Pose, "/Target/pose", self.pose_callback_target, 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback_turtle, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.kill = self.create_client(Kill, '/kill')
        while not self.kill.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/kill service not available, waiting...')
        self.timer_= self.create_timer(0.1, self.go_to_goal)

    def pose_callback_target(self, data2:Pose):
        self.target_pose = data2

    def pose_callback_turtle(self, data1:Pose):
        self.turtle_pose = data1

    def spawn_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Respawned turtle: {response.name}")
        except Exception as e:
            self.get_logger().error(f"Failed to respawn Target turtle: {e}")

    def kill_callback(self, future, turtle_name):
        try:
            response = future.result()
            self.get_logger().info(f'Killed turtle: {turtle_name}')
            self.spawn_target()  # Respawn after killing
        except Exception as e:
            self.get_logger().error(f'Failed to kill turtle {turtle_name}: {e}')

    def spawn_target(self):
        request = Spawn.Request()
        request.x = random.uniform(0.0, 10.0)
        request.y = random.uniform(0.0, 10.0)
        request.theta = random.choice([0.0, 1.57, 3.14, -1.57])
        request.name = "Target"
        try:
            future = self.spawn.call_async(request)
            future.add_done_callback(self.spawn_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to Spawn Turtle Again: {e}")

    def kill_target(self, turtle_name):
        request = Kill.Request()
        request.name = turtle_name
        try:
            self.future = self.kill.call_async(request)
            self.future.add_done_callback(lambda future: self.kill_callback(future, turtle_name))
        except Exception as e:
            self.get_logger().error(f"Failed to kill the Target: {e}")

    def go_to_goal(self):
        cmd = Twist()
        # Ecludian Distance
        dx = self.target_pose.x - self.turtle_pose.x
        dy = self.target_pose.y - self.turtle_pose.y
        distance = math.sqrt( dx**2 + dy**2 )
        if distance < 0.5:
            self.get_logger().info("Target Caught")
            self.kill_target("Target")
            return
        # Angle to Goal
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = angle_to_goal - self.turtle_pose.theta
        try:
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            cmd.linear.x = min(distance,2.0)
            cmd.angular.z = 3.0 * angle_diff
            self.cmd_vel_pub_.publish(cmd)
        except Exception as e:
            self.get_logger().error(f"Failed in reaching the location: {e}")

def main():
    rclpy.init()
    node = Control_Turtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
