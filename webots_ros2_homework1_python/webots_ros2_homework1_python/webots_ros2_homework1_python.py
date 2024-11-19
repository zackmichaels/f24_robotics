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
        self.turtlebot_360 = False
        self.start_yaw = -5.0
        self.prev_yaw = 0.0
        self.total_radians=''
        self.orientation=''
        self.pose_saved=''
        self.cycle_count = 1
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
        self.cmd = Twist()
        self.timer = self.create_timer(timer_period, self.timer_callback)




    def listener_callback1(self, msg1):
        #self.get_logger().info('scan: "%s"' % msg1.ranges)
        self.scan_cleaned = msg1.ranges
       
        #self.get_logger().info('scan: "%s"' % scan)
        # Assume 360 range measurements




    def listener_callback2(self, msg2):
        position = msg2.pose.pose.position
        orientation = msg2.pose.pose.orientation
        (posx, posy, posz) = (position.x, position.y, position.z)
        (qx, qy, qz, qw) = (orientation.x, orientation.y, orientation.z, orientation.w)
        self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved=position
        self.orientation=orientation
       
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

        self.get_logger().info('CYCLE COUNT: "%s"'%  self.cycle_count)




        if self.startx < -800:
            self.startx = 0.0#self.pose_saved.x
            self.starty = 0.0#self.pose_saved.y
            self.prevx = 0.0#self.pose_saved.x
            self.prevy = 0.0#self.pose_saved.y


        if (self.cycle_count % 50 == 0) :
            siny_cosp = 2 * (self.orientation.w * self.orientation.z + self.orientation.x * self.orientation.y)
            cosy_cosp = 1 - 2 * (self.orientation.y * self.orientation.y + self.orientation.z * self.orientation.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)


            if self.start_yaw < -4:
                self.start_yaw = yaw
                self.prev_yaw = yaw
                self.total_radians = 0.0


            delta_yaw = yaw - self.prev_yaw


            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi


            self.total_radians += delta_yaw
            self.prev_yaw = yaw
            if abs(self.total_radians) < (6.28 - 0.0873):
                self.get_logger().info('360TURNING')
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.30
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
            else:
                self.get_logger().info('Angle reached, STOP')
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.total_radians = 0.0
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
                self.cycle_count = self.cycle_count + 1


            # Log the accumulated angular distance


            return
       
        if front_lidar_min < (SAFE_STOP_DISTANCE + 0.35) :


            if (left_lidar_min > right_lidar_min) and (self.turtlebot_turning == False): #and left_lidar_min > front_lidar_min
                self.get_logger().info('left lidar greatest distance!')


                # if left_lidar_min > front_lidar_min:
                #self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.15  # Turn left at 0.5 rad/s
                self.turtlebot_turning = True
                self.random_turn = self.random_turn + 1
                #self.publisher_.publish(self.cmd)
                #self.turtlebot_moving = True
               
            elif (right_lidar_min > left_lidar_min) and (self.turtlebot_turning == False): #and right_lidar_min > front_lidar_min
                self.get_logger().info('right lidar greatest distance!')


                #if right_lidar_min > front_lidar_min:
                #self.cmd.linear.x = 0.0
                self.cmd.angular.z = -0.15 # Turn right at 0.5 rad/s
                self.turtlebot_turning = True
                self.random_turn = self.random_turn + 1
                #self.publisher_.publish(self.cmd)
                #self.turtlebot_moving = True




            self.cmd.linear.x = 0.0
            self.publisher_.publish(self.cmd)
            self.turtlebot_moving = True
            #self.x_y_values.append({'Column 1': self.pose_saved.x, 'Column 2': self.pose_saved.y})
            #df = pd.DataFrame(self.x_y_values)
            #df.to_csv('/mnt/c/CS560/f24_robotics/webots_ros2_homework1_python/webots_ros2_homework1_python/output.csv', index=False)
            # self.get_logger().info('Publishing: "%s"' % self.cmd)

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
                self.cmd.linear.x = 0.22
                self.cmd.angular.z = 0.0
                self.turtlebot_turning = False
                self.cycle_count = self.cycle_count + 1
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True


                self.total_distance = self.total_distance + math.sqrt((self.pose_saved.x - self.prevx)**2 + (self.pose_saved.y - self.prevy)**2)
                self.prevx = self.pose_saved.x
                self.prevy = self.pose_saved.y


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



