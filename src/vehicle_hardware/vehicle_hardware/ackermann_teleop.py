import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import math

settings = termios.tcgetattr(sys.stdin)

msg = """
Ackermann Custom Teleop
---------------------------
Speed Control:
  w : increase forward speed
  s : force stop (and center steering)
  x : increase reverse speed

Steering Control:
  u : increment steering LEFT
  i : center steering (STRAIGHT)
  o : increment steering RIGHT

CTRL-C to quit
"""

class AckermannTeleop(Node):
    def __init__(self):
        super().__init__('ackermann_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_twist)

        # Vehicle State
        self.v_x = 0.0
        self.steering_angle = 0.0

        # Tuning Parameters
        self.speed_step = 0.1
        self.steer_step = 0.05
        self.max_steer = 0.52
        self.L = 0.148 

        print(msg)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_twist(self):
        key = self.getKey()
        if key:
            if key == 'w':
                self.v_x += self.speed_step
            elif key == 'x':
                self.v_x -= self.speed_step
            elif key == 's':
                self.v_x = 0.0
                self.steering_angle = 0.0
            elif key == 'u':
                self.steering_angle += self.steer_step
                if self.steering_angle > self.max_steer:
                    self.steering_angle = self.max_steer
            elif key == 'o':
                self.steering_angle -= self.steer_step
                if self.steering_angle < -self.max_steer:
                    self.steering_angle = -self.max_steer
            elif key == 'i':
                self.steering_angle = 0.0
            elif key == '\x03': # CTRL-C
                rclpy.shutdown()
                return
            
            # Print current state to terminal
            print(f"Speed: {self.v_x:.2f} | Steering Angle: {self.steering_angle:.2f}", end='\r')

        twist_msg = Twist()
        twist_msg.linear.x = self.v_x
        
        if self.v_x != 0.0:
            twist_msg.angular.z = (self.v_x * math.tan(self.steering_angle)) / self.L
        else:
            twist_msg.angular.z = 0.0

        self.pub.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AckermannTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()