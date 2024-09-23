import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import time
from rclpy.duration import Duration
import random
import pandas as pd



LINEAR_VEL = 0.22
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.05
LIDAR_AVOID_DISTANCE = 0.7
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
RIGHT_SIDE_INDEX = 270
RIGHT_FRONT_INDEX = 210
LEFT_FRONT_INDEX=150
LEFT_SIDE_INDEX=90



class RandomWalk(Node):

    def __init__(self):
        # Initialize the publisher
        super().__init__('random_walk_node')
        self.scan_cleaned = []
        self.x_y_values = []
        self.stall = False
        self.startx = -1000.0
        self.starty = -1000.0
        self.prevx = -1000.0
        self.prevy = -1000.0
        self.total_distance = 0.0
        self.furthest_distance = 0.0
        self.turtlebot_moving = False
        self.turtlebot_turning = False
        self.random_turn = 1
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber1 = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback1,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscriber2 = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback2,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.laser_forward = 0
        self.odom_data = 0
        timer_period = 0.5
        self.pose_saved=''
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        scan = msg1.ranges
        self.scan_cleaned = []
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements
        for reading in scan:
            if reading == float('Inf'):
                self.scan_cleaned.append(3.5)
            elif math.isnan(reading):
                self.scan_cleaned.append(0.0)
            else:
            	self.scan_cleaned.append(reading)


    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved=position
        
        #Example of how to identify a stall..need better tuned position deltas; wheels spin and example fast
        #diffX = math.fabs(self.pose_saved.x- position.x)
        #diffY = math.fabs(self.pose_saved.y - position.y)
        #if (diffX < 0.0001 and diffY < 0.0001):
           #self.stall = True
        #else:
           #self.stall = False
           
        return None
        
    def timer_callback(self):
        if (len(self.scan_cleaned)==0):
            self.turtlebot_moving = False
            return
        
        left_lidar_min = min(self.scan_cleaned[LEFT_SIDE_INDEX:LEFT_FRONT_INDEX])
        right_lidar_min = min(self.scan_cleaned[RIGHT_FRONT_INDEX:RIGHT_SIDE_INDEX])
        front_lidar_min = min(self.scan_cleaned[LEFT_FRONT_INDEX:RIGHT_FRONT_INDEX])

        self.get_logger().info('left scan slice: "%s"'%  left_lidar_min)
        self.get_logger().info('front scan slice: "%s"'%  front_lidar_min)
        self.get_logger().info('right scan slice: "%s"'%  right_lidar_min)

        self.get_logger().info('ANGULAR: "%s"'%  right_lidar_min)

        if self.startx < -800:
            self.startx = self.pose_saved.x
            self.starty = self.pose_saved.y
            self.prevx = self.pose_saved.x
            self.prevy = self.pose_saved.y
        
        if front_lidar_min < (SAFE_STOP_DISTANCE + 0.35) :
            
            if (self.random_turn % 8 == 0):
                self.get_logger().info('RANDOM TURN')
                randomNum = random.randint(0,1)
                if (randomNum == 0) and (self.turtlebot_turning == False):
                    self.get_logger().info('RANDOM L')
                    self.cmd.angular.z = 0.1      # Turn left at 0.5 rad/s
                    self.turtlebot_turning = True
                    self.random_turn = self.random_turn + 1
                elif (self.turtlebot_turning == False):
                    self.get_logger().info('RANDOM R')
                    self.cmd.angular.z = -0.1  # Turn right at 0.5 rad/s
                    self.turtlebot_turning = True
                    self.random_turn = self.random_turn + 1

            elif (left_lidar_min > right_lidar_min) and (self.turtlebot_turning == False): #and left_lidar_min > front_lidar_min
                self.get_logger().info('left lidar greatest distance!')

                # if left_lidar_min > front_lidar_min:
                #self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.1  # Turn left at 0.5 rad/s
                self.turtlebot_turning = True
                self.random_turn = self.random_turn + 1
                #self.publisher_.publish(self.cmd)
                #self.turtlebot_moving = True
                
            elif (right_lidar_min > left_lidar_min) and (self.turtlebot_turning == False): #and right_lidar_min > front_lidar_min
                self.get_logger().info('right lidar greatest distance!')

                #if right_lidar_min > front_lidar_min:
                #self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.1 # Turn right at 0.5 rad/s
                self.turtlebot_turning = True
                self.random_turn = self.random_turn + 1
                #self.publisher_.publish(self.cmd)
                #self.turtlebot_moving = True

            self.total_distance = self.total_distance + math.sqrt((self.pose_saved.x - self.prevx)**2 + (self.pose_saved.y - self.prevy)**2)
            self.prevx = self.pose_saved.x
            self.prevy = self.pose_saved.y

            if self.furthest_distance < math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2):
                self.furthest_distance = math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2)

            self.cmd.linear.x = 0.0  
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            #self.x_y_values.append({'Column 1': self.pose_saved.x, 'Column 2': self.pose_saved.y})
            #df = pd.DataFrame(self.x_y_values)
            #df.to_csv('/mnt/c/CS560/f24_robotics/webots_ros2_homework1_python/webots_ros2_homework1_python/output.csv', index=False)
            # self.get_logger().info('Publishing: "%s"' % self.cmd)
            self.get_logger().info('TOTAL DISTANCE : %f' % self.total_distance)
            self.get_logger().info('FURTHEST DISTANCE : %f' % self.furthest_distance)
            # self.get_logger().info(f'x_y_values: {self.x_y_values}')
            return
                    #return
        # elif front_lidar_min < LIDAR_AVOID_DISTANCE:
        #         self.cmd.linear.x = 0.07 
        #         if (right_lidar_min > left_lidar_min):
        #            self.cmd.angular.z = -0.3
            #        else:
        #            self.cmd.angular.z = 0.3
        #         self.publisher_.publish(self.cmd)
        #         self.get_logger().info('Turning')
        #         self.turtlebot_moving = True
        else: # choose furthest available route
                self.get_logger().info('Free range, just runnin')
                self.cmd.linear.x = 0.3
                self.cmd.angular.z = 0.0
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True

                self.total_distance = self.total_distance + math.sqrt((self.pose_saved.x - self.prevx)**2 + (self.pose_saved.y - self.prevy)**2)
                self.prevx = self.pose_saved.x
                self.prevy = self.pose_saved.y

                if self.furthest_distance < math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2):
                    self.furthest_distance = math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2)

                self.get_logger().info('TOTAL DISTANCE : %f' % self.total_distance)
                self.get_logger().info('FURTHEST DISTANCE : %f' % self.furthest_distance)
                #self.x_y_values.append({'Column 1': self.pose_saved.x, 'Column 2': self.pose_saved.y})
                #df = pd.DataFrame(self.x_y_values)
                #df.to_csv('/mnt/c/CS560/f24_robotics/webots_ros2_homework1_python/webots_ros2_homework1_python/output.csv', index=False)
                # self.get_logger().info(f'x_y_values: {self.x_y_values}')
                return

            
        self.get_logger().info('Distance of the obstacle : %f' % front_lidar_min)
        self.get_logger().info('I receive: "%s"' %
                               str(self.odom_data))
        if self.stall == True:
           self.get_logger().info('Stall reported')
        
        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % self.cmd)
 


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    random_walk_node = RandomWalk()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(random_walk_node)
    # Explicity destroy the node
    random_walk_node.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()



if __name__ == '__main__':
    main()
