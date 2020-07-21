import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_system_default

import numpy as np

class Turtlebot3Drive(Node):
    # ノード名
    #SELFNODE = "cmd_vel"
    # トピック名
    TOPIC_VEL = "cmd_vel"
    TOPIC_SCAN = "scan"
    TOPIC_ODOM = "odom"

    def __init__(self):
        # variable define
        self.scan_data = np.zeros(3)
        self.poses = np.zeros(3)
        self.orientations = np.zeros(4)

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

        profile = qos_profile_system_default

        self.scan_sub = self.create_subscription(LaserScan, self.TOPIC_SCAN, self.scan_callback, profile)
        self.odom_sub = self.create_subscription(Odometry, self.TOPIC_ODOM, self.odom_callback, 1)
        #self.scan_sub

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

        #self.get_logger().info("scan0 %lf" % self.scan_data[0])
        #self.get_logger().info("scan1 %lf" % self.scan_data[1])
    
    def odom_callback(self, msg):
        #pass
        self.poses[0] = msg.pose.pose.position.x
        self.poses[1] = msg.pose.pose.position.y
        self.poses[2] = msg.pose.pose.position.z
        self.orientations[0] = msg.pose.pose.orientation.x
        self.orientations[1] = msg.pose.pose.orientation.y
        self.orientations[2] = msg.pose.pose.orientation.z
        self.orientations[3] = msg.pose.pose.orientation.w


        #self.get_logger().info("odom %s" % msg)
        self.get_logger().info("odom %f" % self.poses[0])

    def callback(self):
        """
        タイマーの実行部
        """
        self.get_logger().info("Publish [%s]" % (self.count))
        #self.get_logger().info("hello %f" % (scan_data))

        # 送信するメッセージの作成
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.
        cmd_vel.angular.z = 0.6
        #msg.value = self.count
        
        # 送信
        self.pub.publish(cmd_vel)
        # カウンタをインクリメント
        self.count += 1.

    
        






def main(args=None):
    
    rclpy.init(args=args)

    turtlebot3_drive = Turtlebot3Drive()

    rclpy.spin(turtlebot3_drive)

    turtlebot3_drive.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
