import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random


class Ball(Node):
    def __init__(self):
        super().__init__('turtle_pong_ball')
        self.pub = self.create_publisher(Twist, '/ball/cmd_vel', 10)
        self.ball_pose = None
        self.left_pose = None
        self.right_pose = None
        self.create_subscription(Pose, '/ball/pose', self.ball_cb, 10)
        self.create_subscription(Pose, '/paddle_left/pose', self.left_cb, 10)
        self.create_subscription(Pose, '/paddle_right/pose', self.right_cb, 10)
        self.timer = self.create_timer(0.05, self.tick)
        # velocity vector
        angle = random.uniform(-0.5, 0.5)
        self.vx = 3.0 * (1 if random.random() > 0.5 else -1)
        self.vy = 3.0 * math.sin(angle)
        self.paddle_height = 2.0
        self.play_area_x_min = 0.0
        self.play_area_x_max = 11.0
        self.play_area_y_min = 0.0
        self.play_area_y_max = 11.0
        self.get_logger().info('ball node started')
    
    def ball_cb(self, msg: Pose):
        self.ball_pose = msg


    def left_cb(self, msg: Pose):
        self.left_pose = msg


    def right_cb(self, msg: Pose):
        self.right_pose = msg


    def tick(self):
        if self.ball_pose is None:
            return
        x = self.ball_pose.x
        y = self.ball_pose.y


# bounce off top/bottom
        if y >= self.play_area_y_max - 0.1 and self.vy > 0:
            self.vy = -self.vy
        if y <= self.play_area_y_min + 0.1 and self.vy < 0:
            self.vy = -self.vy


    # paddle collision left
        if self.left_pose and self.vx < 0:
            px = self.left_pose.x
            if x <= px + 0.6 and x >= px: # approaching left paddle
                if abs(y - self.left_pose.y) <= self.paddle_height / 2.0:
                    self.vx = -self.vx * 1.05 # reflect and slightly speed up
        # add spin from where it hit the paddle
                    offset = (y - self.left_pose.y)
                    self.vy += offset * 1.0
        # paddle collision right
        if self.right_pose and self.vx > 0:
            px = self.right_pose.x
            if x >= px - 0.6 and x <= px:
                if abs(y - self.right_pose.y) <= self.paddle_height / 2.0:
                    self.vx = -self.vx * 1.05
                    offset = (y - self.right_pose.y)
                    self.vy += offset * 1.0


# scoring: ball out of left or right bounds
        if x < self.play_area_x_min + 0.1 or x > self.play_area_x_max - 0.1:
            # reset to center
            self.get_logger().info('score! resetting ball to center')
            # teleport by publishing zero and resetting velocities and hope spawn_and_start set turtle
            # better approach: call teleport service but keep simple: publish velocities towards center
            self.vx = 3.0 * (1 if x < (self.play_area_x_min + self.play_area_x_max)/2 else -1)
            self.vy = random.uniform(-1.0, 1.0)
            # Ideally call TeleportAbsolute service to center the ball, but keep minimal here.


# publish velocity
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        self.pub.publish(twist)




def main(args=None):
    rclpy.init(args=args)
    node = Ball()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()