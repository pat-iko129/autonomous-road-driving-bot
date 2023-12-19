#!/usr/bin/env python3
#
#   ros.py
#
#   Set up a very simple ROS node to listen for goals/commands and
#   publish the current pose (position and heading).
#
#   Node:       /PINAME         (this will use your Pi's name)
#
#   Publish:    ~/pose                  geometry_msgs/Pose
#   Subscribe:  ~/goal                  geometry_msgs/Point
#   Subscribe:  ~/explore               std_msgs/Empty
#
import rclpy
import socket
import traceback
import ui as ui
import robot

from math import pi, sin, cos

from rclpy.node                 import Node
from rclpy.time                 import Time, Duration
from geometry_msgs.msg          import Point, Pose
from std_msgs.msg               import Empty


#
#   Global variables
#
#   You may use these global variables, having the robot thread
#   push/write the current coodinates/heading here every cycle.
#
x = 2
y = 4
heading = 6
bot = None

#
#   Simple Node Class
#
class SimpleNode(Node):
    # Initialization.
    def __init__(self, name):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Clear the last sent content (posx,posy,theta) and time.
        self.posx  = None
        self.posy  = None
        self.theta = None
        self.time  = Time()

        # Create the publisher for the pose information.
        self.pub = self.create_publisher(Pose, '~/pose', 10)

        # Then create subscribers for goal and explore commands.
        self.create_subscription(Point, '~/goal',    self.cb_goal,    10)
        self.create_subscription(Empty, '~/explore', self.cb_explore, 10)

        # Finally create a timer to drive the node.
        self.timer = self.create_timer(0.1, self.cb_timer)

        # Report and return.
        self.get_logger().info("ROS Node '%s' running" % (name))

    # Shutdown.
    def shutdown(self):
        # Destroy the timer and shut down the node.
        self.destroy_timer(self.timer)
        self.destroy_node()


    # Timer callback.
    def cb_timer(self):
        # Grab the current x/y and heading.  You may either have the
        # robot thread push the data into the global variables
        # (defined above).  Or you may add a function call here to
        # retrieve the data....

        global x,y,heading
        ui.update_pos(bot)

        # Convert into a global position x, position y, theta (in
        # radians).  Consider the below an example - in particular you
        # might need to change how the angle is computed.
        posx  = float(x)
        posy  = float(y)
        theta = pi/2 + pi/4 * float(heading)

        # Check whether to send (if something changed or 5s have passed).
        now = self.get_clock().now()
        if ((posx != self.posx) or (posy != self.posy) or
            (theta != self.theta) or
            (now - self.time > Duration(seconds=5.0))):

            # Populate the message with the data and send.
            msg = Pose()
            msg.position.x    = posx
            msg.position.y    = posy
            msg.orientation.z = sin(theta/2)
            msg.orientation.w = cos(theta/2)
            self.pub.publish(msg)

            # Retain as the last sent info.
            self.posx  = posx
            self.posy  = posy
            self.theta = theta
            self.time  = now


    # Goal command callback.
    def cb_goal(self, msg):
        # Extract the goal coordinates from the message.
        xgoal = msg.x
        ygoal = msg.y

        # Report.
        self.get_logger().info("Received goal command (%d,%d)" % (xgoal,ygoal))

        # Inject the goal command into your robot thread, just as if
        # you had typed the goal command in the UI thread.
        ui.set_goal(xgoal, ygoal, bot)
        

    # Explore command callback.
    def cb_explore(self, msg):
        # Report.
        self.get_logger().info("Received explore command")
        
        # Inject the explore command into your robot thread, just as
        # if you had typed the explore command in the UI thread.
        ui.ROS_explore(bot)



#
#   Main ROS Thread Code
#
def runros(robot):
    # Initialize ROS.
    rclpy.init()

    # Instantiate the simple node, named after the host name.
    node = SimpleNode(socket.gethostname())
    global bot
    bot = robot
    # Spin the node until interrupted.
    try:
        rclpy.spin(node)
    except BaseException as ex:
        print("Ending runros due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    runros()