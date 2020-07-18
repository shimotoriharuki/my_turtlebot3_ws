import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

scan_data = 0.
global cnt
cnt = 0

class Turtlebot3Sub(Node):
    # ノード名
    #SELFNODE = "cmd_vel"
    # トピック名
    TOPIC_ODOM = "odom"
    TOPIC_SCAN = "scan"

    def __init__(self):
        # ノードの初期化
        super().__init__('py_turtlebot3_sub')

        # コンソールに表示
        self.get_logger().info("%s initializing..." % (self.TOPIC_SCAN))

        self.scan_odom = self.create_subscription(Odometry, self.TOPIC_ODOM, self.odom_callback, 10)
        self.scan_scan = self.create_subscription(LaserScan, self.TOPIC_SCAN, self.scan_callback, 10)

        self.create_timer(0.5, self.timer_callback)

        # コンソールに表示
        self.get_logger().info("%s do..." % self.TOPIC_SCAN)

    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        self.get_logger().info("%s done." % self.TOPIC_SCAN)
    
    def odom_callback(self, msg):
        #scan_data = LaserScan()
  
        self.get_logger().info("odom")
        
    def scan_callback(self, msg):
        #scan_data = LaserScan()
  
        self.get_logger().info("scan")


    def timer_callback(self):
        global cnt
        cnt = cnt + 1
        self.get_logger().info("cnt %s" % cnt)

def main(args=None):
    
    rclpy.init(args=args)

    turtlebot3_sub = Turtlebot3Sub()

    rclpy.spin(turtlebot3_sub)

    turtlebot3_sub.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
