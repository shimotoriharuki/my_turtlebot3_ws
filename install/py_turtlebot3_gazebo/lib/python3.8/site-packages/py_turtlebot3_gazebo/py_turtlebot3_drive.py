import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_system_default

import numpy as np

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation

class Turtlebot3Drive(Node):
    # ノード名
    #SELFNODE = "cmd_vel"
    # トピック名
    TOPIC_VEL = "cmd_vel"
    TOPIC_SCAN = "scan"
    TOPIC_ODOM = "odom"
    TOPIC_MARKER = "marker"

    def __init__(self):
        # variable define
        self.scan_data = np.zeros(3)
        self.poses = np.zeros(3)
        self.orientations = np.zeros(4)

        self.cova_l = np.zeros(3)
        self.cova_a = np.zeros(3)
        self.time = 0

        self.nu = 0.1
        self.omega = 0.05

        self.ptheta = 0
        self.px = 0
        self.py = 0

        # ノードの初期化
        super().__init__(self.TOPIC_VEL)
        super().__init__(self.TOPIC_SCAN)
        super().__init__(self.TOPIC_ODOM)

        # コンソールに表示
        self.get_logger().info("%s initializing..." % (self.TOPIC_VEL))
        self.get_logger().info("%s initializing..." % (self.TOPIC_SCAN))
        self.get_logger().info("%s initializing..." % (self.TOPIC_ODOM))

        # publisherインスタンスを生成
        self.pub = self.create_publisher(Twist, self.TOPIC_VEL, 10)

        # subscriberインスタンスを生成
        profile = qos_profile_system_default
        self.scan_sub = self.create_subscription(LaserScan, self.TOPIC_SCAN, self.scan_callback, profile)
        self.odom_sub = self.create_subscription(Odometry, self.TOPIC_ODOM, self.odom_callback, 1)

        # タイマーのインスタンスを生成（1秒ごとに発生）
        self.create_timer(1, self.callback)

        # カウンタをリセット
        self.count = 0

        # コンソールに表示
        self.get_logger().info("%s do..." % self.TOPIC_VEL)

    def __del__(self):
        """
        デストラクタ
        """
        # コンソールに表示
        #     
    def scan_callback(self, msg):

        self.scan_data[0] = msg.ranges[0]
        self.scan_data[1] = msg.ranges[1]
    
    def odom_callback(self, msg):
        #pass
        self.poses[0] = msg.pose.pose.position.x
        self.poses[1] = msg.pose.pose.position.y
        self.poses[2] = msg.pose.pose.position.z
        self.orientations[0] = msg.pose.pose.orientation.x
        self.orientations[1] = msg.pose.pose.orientation.y
        self.orientations[2] = msg.pose.pose.orientation.z
        self.orientations[3] = msg.pose.pose.orientation.w

        self.cova_l[0] = msg.twist.twist.linear.x
        self.cova_l[1] = msg.twist.twist.linear.y
        self.cova_l[2] = msg.twist.twist.linear.z

    def callback(self):
        """
        タイマーの実行部
        """
        #--------------cmd_velo publish----------------#
        # 送信するメッセージの作成
        cmd_vel = Twist()
        cmd_vel.linear.x = self.nu
        cmd_vel.angular.z = self.omega
        
        # 送信
        self.pub.publish(cmd_vel)

        self.ptheta = self.omega * self.time
        self.get_logger().info("theta %f" % self.ptheta)
        self.time = self.time + 0.1

def main(args=None):
    
    rclpy.init(args=args)

    turtlebot3_drive = Turtlebot3Drive()

    rclpy.spin(turtlebot3_drive)

    turtlebot3_drive.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
