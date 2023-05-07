"""
    new algorith, much simpler 
"""

from math import pi
import math
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class Follow(Node):
    def __init__(self):
        super().__init__('follow')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE



        self.zero = 0.0 # index of 0-degree LIDAR ray
        self.zero_range = np.Inf # range of 0-degree LIDAR ray
        self.twoseventy_range = np.Inf # range of 270-degree LIDAR ray
        self.ninty_range = 0.0 # range of 90-degree LIDAR ray
        self.angle_increment = 1.0

        #def want to sort through all of these and figure out which ones we are actually using at some point
        self.desired_wall_distance = 0.3
        self.wall_distance = 0.0 # 0-degree range before rotate so parallel state
        self.state = 'follow wall' # current state of robot

        self.goal_angle = 90 # goal angle for minimum distance from wall

        self.index_min_dist = 0.0
        self.angle_min_dist = 0.0

        self.turn = 0.0 # angle has turned so far (approx)
        self.drive = 0.0 # distance have traveled so far (approx)
        
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.laser_listener_callback,
            qos_profile,
        )

        self.timer_period = 0.05 # update time in seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.msg = Twist()
        
    def getch(self, timeout=0.1): #detecting keyboard shenanagins
        import sys, tty, termios, select
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            rlist, _, _ = select.select([sys.stdin], [], [], timeout)
            if rlist:
                ch = sys.stdin.read(1)
            else:
                ch = None
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def angle(self, angle_):  # returns the index of a given angle
        ninty = self.zero + int((pi*angle_/180)/ self.angle_increment)
        return int(ninty)
    
    def indexToAngle(self, index):  # returns the angle of a given index
        angle = ((index - self.zero)) *  self.angle_increment * 180/pi
        if angle < 0:
            angle = (angle + 360) % 360
        return angle
    
    def timer_callback(self):

        # i decided to use shorter variable names and then assign them to the message variables later
        lin_x = 0.0
        ang_z = 0.0

        ### obstacle detection section
        obstacleRange = 0.5 #sets sensetivity to how close a wall has to be for it to detect an object

        #initializes obstacle bools
        obstacleRight = False  
        obstacleFront = False
        obstacleLeft = False

        #sets obstacle bools to true if theres an obstacle detected within the obstacle range
        if (self.zero_range < obstacleRange):
            obstacleFront = True
        if (self.ninty_range < obstacleRange):
            obstacleLeft = True
        if (self.twoseventy_range < obstacleRange):
            obstacleRight = True


        ###path finding section

        #self.state is only used for outputting debug messages

        #if theres something in front of the robot, then it has to turn somewhere.  
        if (obstacleFront):
            lin_x = 0.0
            if(not obstacleLeft):  #if theres nothing to the left of it, it will turn left
                ang_z = 0.3
                self.state = "turn left"
            elif(not obstacleRight): # if theres nothing to the right of it or to the left of it, it will turn right
                ang_z = -0.3
                self.state = "turn right"
            #wasnt sure about the order of precedence with the above two conditions.  
            # currently it prefers turning left, but might want to swap them.  further testing required

            #edge cases, getting out of odd spots where it spins but there is a clear path
            #1.4 was picked because right triangles and the distance to a corner if its coming up on a dead end
            elif(self.frontright_range > obstacleRange*1.4):
                lin_x = 0.1
                ang_z = -0.2
                self.state = "angling right"
            elif(self.frontleft_range > obstacleRange*1.4):
                lin_x = 0.1
                ang_z = 0.2
                self.state = "angling left"
            # worst case just keep spinning (e.g. dead end)
            else: 
                lin_x = 0.0
                ang_z = 0.3
                self.state = "just keep turning left..."

        #if theres no obstacle in front, and there is an obstacle to the right, itll continue following that wall
        elif(obstacleRight):
            lin_x = 0.25
            ang_z = 0.0
            self.state = "follow wall"

            # correcting, if its turning a corner and didnt quite turn all the way then this will make sure it keeps turning
            if(self.frontright_range < self.twoseventy_range*1.4 and (self.twoseventy_range < 0.3 or self.frontright_range < 0.3)):
                lin_x = 0.0
                ang_z = 0.25
                self.state = "correcting"

            #if theres a wall that its following and its too far away, itll go closer
            if(self.twoseventy_range > 0.35):
                ang_z = -1*self.twoseventy_range/4
                self.state = "too far, steering towards"
            
            #if its following a wall and gets too close to a wall on the other side, itll correct
            if(self.frontleft_range < self.ninty_range*1.414 and (self.ninty_range < 0.3 or self.frontleft_range < 0.3)):
                lin_x = 0.1
                ang_z = -0.2
                self.state = "correcting, left side"

        #if theres no obstacle in front and theres an obstacle to the left itll go into find-a-wall mode
        #this is either if its just starting out or if it just lost the right wall and theres no wall in front
        elif(obstacleLeft):
            lin_x = 0.12
            ang_z = -0.4
            self.state = "steering right"
        else: #in the void
            lin_x = 0.2
            ang_z = -0.2
        
        # outputs debugging info
        self.get_logger().info(str(obstacleLeft) + "  " + str(obstacleFront) + "  " + str(obstacleRight) + '\t' + self.state)

        
        self.msg.linear.x = lin_x
        self.msg.angular.z = ang_z
        
        key = self.getch()
        if key == 'w':
            self.msg.linear.x = 0.2  
        elif key == 'x':
            self.msg.linear.x = -0.2
        elif key == 'd':
            self.msg.angular.z = 0.2
        elif key == 'a':
            self.msg.angular.z = -0.2
        elif key == ' ':
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0
            self.state = ''
        
        self.publisher_.publish(self.msg)

    def laser_listener_callback(self, msg):

        self.angle_increment = msg.angle_increment
        self.zero = int((0.0 - msg.angle_min) / self.angle_increment) # finds index of 0 degree LIDAR ray

        msg.ranges = [msg.range_max if i > msg.range_max else i for i in msg.ranges] # makes sure no LIDAR ray is above range max

        # averages values +- 5 degrees around target LIDAR
        self.zero_range = np.mean([msg.ranges[self.angle(i)] for i in list(range(0, 6)) + list(range(355, 361))])
        self.ninty_range = np.mean([msg.ranges[self.angle(i)] for i in range(85, 96)])
        self.twoseventy_range = np.mean([msg.ranges[self.angle(i)] for i in range(260, 280)])

        self.frontright_range = np.mean([msg.ranges[self.angle(i)] for i in range(305, 325)])
        self.frontleft_range = np.mean([msg.ranges[self.angle(i)] for i in range(35, 55)])


        ###dont need?###
        # find the angle of the closest distance to the wall.  
        # if parallel is 90.  if > 90, should rotate CW.  if < 90, should rotate CCW
        #left_ranges = msg.ranges[self.angle(45) : self.angle(135)]
        self.wall_distance = min(msg.ranges)

        self.index_min_dist = msg.ranges.index(self.wall_distance)
        self.angle_min_dist = self.indexToAngle(self.index_min_dist)
        #################

        #debugging outputs
        self.get_logger().info(str(round(self.twoseventy_range,2)) + '\t' + str(round(self.zero_range,2)) + '\t' + str(round(self.ninty_range,2)) + '\t' + str(self.state))


def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()