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
from scipy.stats import multivariate_normal

class ParticlesCtrl(Node): 

    def __init__(self):
        # variable define
        self.TOPIC_VEL = "cmd_vel"
        self.TOPIC_PARTICLES = "particles_pose"

        self.PARTICLES_NUM = 5
        self.DELAT_T = 1
        self.V = {"nn":0.19, "no":0.001, "on":0.13, "oo":0.2}
        self.C = np.diag([self.V["nn"] ** 2, self.V["no"] ** 2, self.V["on"] ** 2, self.V["oo"] ** 2])

        self.nu = 0
        self.omega = 0

        self.ideal_theta = 0
        self.ideal_x = -2.0
        self.ideal_y = -0.5

        self.particles_theta = np.zeros(self.PARTICLES_NUM)
        self.particles_x = np.full(self.PARTICLES_NUM, -2.0)
        self.particles_y = np.full(self.PARTICLES_NUM, -0.5)

        self.pre_partcles_theta = np.zeros(self.PARTICLES_NUM)
        self.pre_partcles_x = np.zeros(self.PARTICLES_NUM)
        self.pre_partcles_y = np.zeros(self.PARTICLES_NUM)

        super().__init__(self.TOPIC_VEL)
        super().__init__(self.TOPIC_PARTICLES)
        
        # sub initiarize
        self.get_logger().info("%s initializing..." % (self.TOPIC_VEL))
        self.vel_sub = self.create_subscription(Twist, self.TOPIC_VEL, self.vel_callback, 10)

        # pub initiarize
        self.particles_pub = self.create_publisher(MarkerArray, self.TOPIC_PARTICLES, 10)

        # タイマーのインスタンスを生成
        self.create_timer(self.DELAT_T, self.timer_callback)

    def vel_callback(self, msg):
        # get nu and omega from msg
        self.nu = msg.linear.x
        self.omega = msg.angular.z

        # calc ideal pose with Dead Reckoning
        self.ideal_theta += self.omega * self.DELAT_T
        self.ideal_x += self.nu * self.DELAT_T * math.cos(self.ideal_theta)
        self.ideal_x += self.nu * self.DELAT_T * math.sin(self.ideal_theta)
        self.get_logger().info("theta: %f" % self.ideal_theta)

    def timer_callback(self):
        # particles publish
        marker_data = Marker()
        marker_array = MarkerArray()
        

        for i in range(self.PARTICLES_NUM):

            # add noise
            noise_rate_pdf = multivariate_normal(cov = self.C)
            ns = noise_rate_pdf.rvs()
            noised_nu = self.nu + ns[0] * math.sqrt(abs(self.nu) / self.DELAT_T) + ns[1] * math.sqrt(abs(self.omega) / self.DELAT_T)
            noised_omega = self.omega + ns[2] * math.sqrt(abs(self.nu) / self.DELAT_T) + ns[3] * math.sqrt(abs(self.omega) / self.DELAT_T)

            # calc particles pose with Dead Reckoning
            self.particles_theta[i] = self.pre_partcles_theta[i] + noised_omega * self.DELAT_T
            #self.particles_x[i] = self.pre_partcles_x[i] + noised_nu * self.DELAT_T * math.cos(self.particles_theta[i])
            #self.particles_y[i] = self.pre_partcles_y[i] + noised_nu * self.DELAT_T * math.sin(self.particles_theta[i])
            self.particles_x[i] += noised_nu * self.DELAT_T * math.cos(self.particles_theta[i])
            self.particles_y[i] += noised_nu * self.DELAT_T * math.sin(self.particles_theta[i])

            # store poses into pre variables
            self.pre_partcles_theta[i] = self.particles_theta[i]
            self.pre_partcles_x[i] = self.particles_x[i]
            self.pre_partcles_y[i] = self.particles_y[i]

            # set markers parameters
            rot = Rotation.from_rotvec(np.array([0, 0, self.particles_theta[i]]))
            quo = rot.as_quat()

            marker_data.header.frame_id = "odom"
            #marker_data.header.stamp = rclpy.time

            marker_data.ns = "basic_shapes"
            marker_data.id = i

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = self.particles_x[i]
            marker_data.pose.position.y = self.particles_y[i]
            marker_data.pose.position.z = 0.

            marker_data.pose.orientation.x = quo[0]
            marker_data.pose.orientation.y = quo[1]
            marker_data.pose.orientation.z = quo[2]
            marker_data.pose.orientation.w = quo[3]

            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 1.0
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

