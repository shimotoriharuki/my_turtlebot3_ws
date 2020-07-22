import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_system_default
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from builtin_interfaces.msg import Duration
from scipy.spatial.transform import Rotation
import numpy as np
import math

class ParticlesCtrl(Node): 

    def __init__(self):
        # variable define
        self.topic_vel = "cmd_vel"
        self.topic_particles = "particles_pose"
        self.dt = 0.1
        self.theta = 0
        self.x = -2.0
        self.y = -0.5

        super().__init__(self.topic_vel)
        super().__init__(self.topic_particles)
        
        # sub initiarize
        self.get_logger().info("%s initializing..." % (self.topic_vel))
        self.vel_sub = self.create_subscription(Twist, self.topic_vel, self.vel_callback, 10)

        # pub initiarize
        self.particles_pub = self.create_publisher(MarkerArray, self.topic_particles, 10)

        # タイマーのインスタンスを生成
        self.create_timer(1, self.timer_callback)

    def vel_callback(self, msg):
        self.nu = msg.linear.x
        self.omega = msg.angular.z

        self.theta += self.omega * self.dt
        self.x += self.nu * self.dt * math.cos(self.theta)
        self.y += self.nu * self.dt * math.sin(self.theta)

        #self.get_logger().info("cmd_vel %s" % msg)
        #self.get_logger().info("nu: %f" % self.nu + ", omega: %f" % self.omega)
        self.get_logger().info("theta: %f" % self.theta)

    
    def timer_callback(self):
        # particles publish
        marker_data = Marker()
        marker_array = MarkerArray()
        rot = Rotation.from_rotvec(np.array([0, 0, self.theta]))
        quo = rot.as_quat()


        for i in range(1):
            marker_data.header.frame_id = "odom"
            #marker_data.header.stamp = rclpy.time

            marker_data.ns = "basic_shapes"
            marker_data.id = i

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = self.x
            marker_data.pose.position.y = self.y
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
    
            self.particles_pub.publish(marker_array)



def main(args=None):
    rclpy.init(args=args)
    particles_ctrl = ParticlesCtrl()
    rclpy.spin(particles_ctrl)
    particles_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

