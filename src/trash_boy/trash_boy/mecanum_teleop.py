#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

msg = """
Mecanum Omnidirectional Control
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

u/o : forward + strafe left/right (diagonal)
i   : forward
j/l : strafe left/right
m/. : backward + strafe left/right (diagonal)
,   : backward
k   : stop

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),    # forward
    'o': (1, -1, 0, 0),   # forward + right
    'j': (0, 1, 0, 0),    # left
    'l': (0, -1, 0, 0),   # right
    'u': (1, 1, 0, 0),    # forward + left
    ',': (-1, 0, 0, 0),   # backward
    '.': (-1, -1, 0, 0),  # backward + right
    'm': (-1, 1, 0, 0),   # backward + left
    'O': (1, -1, 0, 1),   # forward + right + rotate right
    'I': (1, 0, 0, 0),    # forward
    'J': (0, 1, 0, -1),   # left + rotate left
    'L': (0, -1, 0, 1),   # right + rotate right
    'U': (1, 1, 0, -1),   # forward + left + rotate left
    '<': (-1, 0, 0, 0),   # backward
    '>': (-1, -1, 0, 1),  # backward + right + rotate right
    'M': (-1, 1, 0, -1),  # backward + left + rotate left
    't': (0, 0, 0, 1),    # rotate right
    'T': (0, 0, 0, -1),   # rotate left
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (0.9, 0.9),
    'w': (1.1, 1),
    'x': (0.9, 1),
    'e': (1, 1.1),
    'c': (1, 0.9),
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return f"currently:\tspeed {speed:.2f}\tturn {turn:.2f}"

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rclpy.init()
    node = Node('mecanum_teleop')
    pub = node.create_publisher(Twist, 'cmd_vel', 10)

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            elif key == 'k' or key == 'K':
                x = 0.0
                y = 0.0
                z = 0.0
                th = 0.0
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()