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
        self.omega = 0.5

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
        self.marker_publisher = self.create_publisher(MarkerArray, self.TOPIC_MARKER, 10)

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

        self.cova_l[0] = msg.twist.twist.linear.x
        self.cova_l[1] = msg.twist.twist.linear.y
        self.cova_l[2] = msg.twist.twist.linear.z

        #self.time = msg.header.stamp.sec

        


        #self.get_logger().info("odom %s" % msg)
        #self.get_logger().info("odom %f" % self.poses[0])
        self.get_logger().info("x %f" % self.cova_l[0]+ "y %f" % self.cova_l[1] + "z %f" % self.cova_l[2])
        self.get_logger().info("time %d" % self.time)

    def callback(self):
        """
        タイマーの実行部
        """
        #self.get_logger().info("Publish [%s]" % (self.count))
        #self.get_logger().info("hello %f" % (scan_data))

        #--------------cmd_velo publish----------------#
        # 送信するメッセージの作成
        cmd_vel = Twist()
        cmd_vel.linear.x = self.nu
        cmd_vel.angular.z = self.omega
        
        # 送信
        self.pub.publish(cmd_vel)
        # カウンタをインクリメント
        #self.count += 1.


        #---------------particle publich--------------#
        marker_data = Marker()
        marker_array = MarkerArray()
        rot = Rotation.from_rotvec(np.array([0, 0, self.ptheta]))
        quo = rot.as_quat()

        
        self.ptheta = self.omega * self.time
        #self.x = self.x + self.nu * self.time
        self.time = self.time + 1

        for i in range(5):
            marker_data.header.frame_id = "odom"
            #marker_data.header.stamp = rclpy.time

            marker_data.ns = "basic_shapes"
            marker_data.id = i

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = 0.0
            marker_data.pose.position.y = 0.0
            marker_data.pose.position.z = float(i)

            marker_data.pose.orientation.x = quo[0]
            marker_data.pose.orientation.y = quo[1]
            marker_data.pose.orientation.z = quo[2]
            marker_data.pose.orientation.w = quo[3]

            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0

            marker_data.scale.x = 0.5
            marker_data.scale.y = 0.05
            marker_data.scale.z = 0.05

            marker_data.lifetime = Duration()

            marker_data.type = 0

            marker_array.markers.append(marker_data)
    
            self.marker_publisher.publish(marker_array)

    
        






def main(args=None):
    
    rclpy.init(args=args)

    turtlebot3_drive = Turtlebot3Drive()

    rclpy.spin(turtlebot3_drive)

    turtlebot3_drive.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()
