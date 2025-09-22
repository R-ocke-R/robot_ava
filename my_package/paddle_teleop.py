import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


import sys
import select
import termios
import tty




class PaddleTeleop(Node):
    def __init__(self):
        super().__init__('paddle_teleop')
        self.left_pub = self.create_publisher(Twist, '/paddle_left/cmd_vel', 10)
        self.right_pub = self.create_publisher(Twist, '/paddle_right/cmd_vel', 10)
        self.get_logger().info('paddle_teleop ready: w/s for left, i/k for right, q to quit')


    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    ch = sys.stdin.read(1)
                    if ch == 'q':
                        break
                    twist_l = Twist()
                    twist_r = Twist()
                    speed = 2.0
                    if ch == 'w':
                        twist_l.linear.y = speed
                    elif ch == 's':
                        twist_l.linear.y = -speed
                    elif ch == 'i':
                        twist_r.linear.y = speed
                    elif ch == 'k':
                        twist_r.linear.y = -speed
                    else:
                    # stop on any other key
                        pass
                    # publish short burst: turtlesim interprets cmd_vel continuously,
                    # to make paddle move smoothly we publish repeatedly in a small loop
                    self.left_pub.publish(twist_l)
                    self.right_pub.publish(twist_r)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)




def main(args=None):
    rclpy.init(args=args)
    node = PaddleTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()




if __name__ == '__main__':
    main()