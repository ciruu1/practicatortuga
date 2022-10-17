#!/usr/bin/env python3

import sys
import termios
import tty
import rclpy
import select

from geometry_msgs.msg import Twist
from rclpy.node import Node
from pynput.keyboard import Key

import turtlesim.srv
from turtlesim.srv import _set_pen
import std_srvs.srv
from std_srvs.srv import _empty


settings= termios.tcgetattr(sys.stdin)

msg = """"
Lee del teclado y lo publica en Twist
--------------------------------------
Moverte:
q   w   e
a       d
z   s   c

--------------------------------------
Velocidad:
u/j = aumentar/disminuir velocidad maxima x10%
i/k = aumentar/disminuir solo velocidad linear x10%
o/l = aumentar/disminuir solo velocidad angular x10%

--------------------------------------
Control:
c= borra lo dibujado
r= reinicia la posicion
SPACE= habilita/deshabilita pintado

"""

moveBindings={
   'q':(1,0,0,1),
   'w':(1,0,0,0),
   'e':(1,0,0,-1),
   'a':(0,0,0,1),
   'z':(-1,0,0,-1),
   's':(-1,0,0,0),
   'x':(-1,0,0,1),
   'd':(0,0,0,-1)
  }

speedBindings={
    'u':(1.1,1.1),
    'j':(.9,.9),
    'i':(1.1,1.1),
    'k':(.9,1),
    'o':(1,1.1),
    'l':(1,.9)
  }
   
controlBindings={
    'c':0,
    'r':0,
    ' ':-1
}

def getKey():
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin],[],[],0)
        key= sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s" %(speed,turn)

def createRequestEmpty(node):
    client = node.create_client(turtlesim.srv, 'empty')
    while not client.wait_for_service(1.0):
        node.get_logger().warn('Waiting for server')
    future = client.call_async(std_srvs.srv.Empty.Request)

def main (args=None):
    if args is None:
        args=sys.argv
    rclpy.init()
    node= rclpy.create_node('teleop_turtle_movement')
    publi= node.create_publisher(Twist, '/turtle1/cmd_vel', 10)



    
    speed=0.5
    turn=1.0
    x=0
    y=0
    z=0
    th=0
    status=0
    color=1
    clear=0
    reset=0

    try:
        print(msg)
        print(vels(speed,turn))
        while(1):
            key=getKey()
            if key in moveBindings.keys():
                x=moveBindings[key][0]
                y=moveBindings[key][1]
                z=moveBindings[key][2]
                th=moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed*speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed, turn))
                if(status == 14):
                    print(msg)
                    status = (status +1)%15
            elif key in controlBindings.keys():
                clear=controlBindings['c']
                reset=controlBindings['r']
                color=color*controlBindings[' ']
                if clear :
                    createRequestEmpty(node)
                elif color==1:
                    turtlesim.srv.SetPen.Request.r=255;turtlesim.srv.SetPen.Request.b=0;turtlesim.srv.SetPen.Request.g=0
                elif color==-1:
                    turtlesim.srv.SetPen.Request.r=0;turtlesim.srv.SetPen.Request.b=0;turtlesim.srv.SetPen.Request.g=0
                elif reset:
                   std_srvs.srv.Empty.Request
            else :
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break
            
            twist=Twist()
            twist.linear.x=x*speed
            twist.linear.y=y*speed
            twist.linear.z=z*speed
            
            twist.angular.x=0.0
            twist.angular.y=0.0
            twist.angular.z=th*turn
            publi.publish(twist)

    except:
        print("Fallo")
    
    finally:
        twist=Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        publi.publish(twist)
        
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown


if __name__ == '__main__':
    main()