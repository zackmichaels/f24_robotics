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
        self.distance = True # Used to switch between angular/distance
        self.startx = -1000.0
        self.starty = -1000.0
        self.desired_distance = 1.0 # Enter meter distance you want traveled
        self.desired_angular = 3.14 # Enter radians turned
        self.speed = 0.15 # Enter linear speed for robot m/s -> .15 or .075
        self.angular_speed = 0.5236 # Angular speed r/s -> 0.5236 for 30 dps | 2.094 for 120 dps
        self.orientation=''
        self.start_yaw = -5
        self.total_radians=''
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
        #self.get_logger().info('self position: {},{},{}'.format(posx,posy,posz))
        # similarly for twist message if you need
        self.pose_saved=position
        self.orientation=orientation
           
        return None
        
    def timer_callback(self):

        if self.startx < -800:
            self.startx = self.pose_saved.x
            self.starty = self.pose_saved.y
            self.prevx = self.pose_saved.x
            self.prevy = self.pose_saved.y

        if self.distance: # Used to switch between straight and turning
            if math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2) < (self.desired_distance - 0.05) :
                self.get_logger().info('LESS THAN 1M, CONTINUE')
                self.cmd.linear.x = self.speed
                self.cmd.angular.z = 0.0
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
            else:
                self.get_logger().info('1M REACHED, STOP')
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True

            self.get_logger().info('DISTANCE: "%s"'%  math.sqrt((self.startx - self.pose_saved.x)**2 + (self.starty - self.pose_saved.y)**2))
        else:
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
            if abs(self.total_radians) < (self.desired_angular - 0.0873):
                self.get_logger().info('TURNING')
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = self.angular_speed
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True
            else:
                self.get_logger().info('Angle reached, STOP')
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.0
                self.turtlebot_turning = False
                self.publisher_.publish(self.cmd)
                self.turtlebot_moving = True

            # Log the accumulated angular distance
            self.get_logger().info('TOTAL ANGULAR DISTANCE: "%s"' % abs(self.total_radians))
        


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
