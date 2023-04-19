"""
    Minimal code for wall follower 
"""

from math import pi

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        self.zero = 0.0
        self.angle_increment = 1.0
        
        self.subscription= self.create_subscription(
            LaserScan,
            '/scan',  ## Read
            self.listener_callback,
            qos_profile,
        )
        timer_period = 0.05 # seconds
        self.i = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.msg = Twist()
        
        
    def getch(self, timeout=0.1):
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

    def angle(self,angle_):
        ninty = self.zero + int((pi*angle_/180)/ self.angle_increment)
        return int(ninty)
    
    def timer_callback(self):
        '''
        Publisher callback function
        TODO: implement
        '''
        # msg = Twist()
        key = self.getch()
        if key == 'w':
            self.msg.linear.x = 1.0  
        elif key == 'x':
            self.msg.linear.x = -1.0  
        elif key == ' ':
            self.msg.linear.x = 0.0
        self.publisher_.publish(self.msg)
        
        
    

    def listener_callback(self,msg):
        '''
        Subscription Callback 
        TODO: implement
        '''
        self.angle_increment = msg.angle_increment
        self.zero = int((0.0 - msg.angle_min) / self.angle_increment)
        
        ninty = self.angle(90)
        
        self.get_logger().info('I heard : Range[0] "%f" Ranges[90]: "%f"' %(msg.ranges[self.zero] ,msg.ranges[ninty] ))



def main(args=None):
    rclpy.init(args=args)
    my_follower = Follow()
    rclpy.spin(my_follower)
    my_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()