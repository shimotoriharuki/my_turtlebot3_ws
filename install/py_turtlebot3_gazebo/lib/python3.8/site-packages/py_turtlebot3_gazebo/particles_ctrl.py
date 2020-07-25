import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
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
import pandas as pd
import random
import copy     

class Particle:
    def __init__(self, init_pose, weight):
        self.pose = init_pose
        self.weight = weight

    def motion_update(self, nu, omega, time, noise_rate_pdf): 
        ns = noise_rate_pdf.rvs()
        pnu = nu + ns[0]*math.sqrt(nu/time) + ns[1]*math.sqrt(omega/time)
        pomega = omega + ns[2]*math.sqrt(nu/time) + ns[3]*math.sqrt(omega/time)
        #self.pose = IdealRobot.state_transition(pnu, pomega, time, self.pose)

    def observation_update(self, observation, envmap, distance_dev_rate, direction_dev):
        for d in observation:
            obs_pos = d[0]
            obs_id = d[1]
            
            ###パーティクルの位置と地図からランドマークの距離と方角を算出###
            #pos_on_map = envmap.landmarks[obs_id].pos
            #particle_suggest_pos = IdealCamera.observation_function(self.pose, pos_on_map)
            
            ###尤度の計算###
            #distance_dev = distance_dev_rate*particle_suggest_pos[0]
            #cov = np.diag(np.array([distance_dev**2, direction_dev**2]))
            #self.weight *= multivariate_normal(mean=particle_suggest_pos, cov=cov).pdf(obs_pos)


class ParticlesCtrl(Node): 
    def __init__(self):
        # variable define
        self.TOPIC_VEL = "cmd_vel"
        self.TOPIC_PARTICLES = "particles_pose"
        self.TOPIC_SCAN = "scan"

        self.PARTICLES_NUM = 50
        self.DELAT_T = 1
        self.V = {"nn":3.16e-3, "no":0.001, "on":3.16e-3, "oo":0.0316}
        self.C = np.diag([self.V["nn"] ** 2, self.V["no"] ** 2, self.V["on"] ** 2, self.V["oo"] ** 2])

        self.nu = 0
        self.omega = 0

        self.ideal_theta = 0
        self.ideal_x = 0.
        self.ideal_y = 0.

        self.particles_theta = np.zeros(self.PARTICLES_NUM)
        self.particles_x = np.full(self.PARTICLES_NUM, 0.)
        self.particles_y = np.full(self.PARTICLES_NUM, 0.)
        self.weights = np.full(self.PARTICLES_NUM, 1.0 / self.PARTICLES_NUM)

        #init_pose = [-2.0, 0, 0]
        #self.particles = [Particle(init_pose, 1.0 / self.PARTICLES_NUM) for i in range(self.PARTICLES_NUM)]

        #self.pre_partcles_theta = np.zeros(self.PARTICLES_NUM)
        #self.pre_partcles_x = np.zeros(self.PARTICLES_NUM)
        #self.pre_partcles_y = np.zeros(self.PARTICLES_NUM)

        self.test_x = 0.
        self.test_y = 0.
        self.test_z = 0.

        self.scan_data = [0]
        self.angle_increment = 0.
        self.store_randmark_pos = []
        self.STORE_NUM = 0

        super().__init__(self.TOPIC_VEL)
        super().__init__(self.TOPIC_PARTICLES)
        super().__init__(self.TOPIC_SCAN)
        
        # sub initiarize
        self.get_logger().info("initializing...")
        self.vel_sub = self.create_subscription(Twist, self.TOPIC_VEL, self.vel_callback, 10)
        profile = qos_profile_system_default
        self.scan_sub = self.create_subscription(LaserScan, self.TOPIC_SCAN, self.scan_callback, profile)

        # pub initiarize
        self.particles_pub = self.create_publisher(MarkerArray, self.TOPIC_PARTICLES, 10)

        # タイマーのインスタンスを生成
        self.create_timer(self.DELAT_T, self.timer_callback)

        self.get_logger().info("waiting /cmd_vel topic...")

    def vel_callback(self, msg):
        # get nu and omega from msg
        self.nu = msg.linear.x
        self.omega = msg.angular.z

        #self.get_logger().info("Subscribed cmd_vel")

    def scan_callback(self, msg):
        self.scan_data = msg.ranges
        self.angle_increment = msg.angle_increment
        #self.get_logger().info("scan: %f" % self.scan_data[0])

    def randmark_position(self, ranges, angle_increment):
        l = min(ranges)
        index = ranges.index(l)
        phi = index * angle_increment
        
        # Normalize to range that -pi ~ pi
        while phi >= np.pi:
            phi -= 2 * np.pi
        while phi < -np.pi:
            phi += 2 * np.pi        
        
        return [l, phi]

    def timer_callback(self):
        # ---------- randmark------------- #
        randmark_pos = self.randmark_position(self.scan_data, self.angle_increment)
        self.get_logger().info("l: %f" % randmark_pos[0] + ", phi: %f" % np.rad2deg(randmark_pos[1]))
        """ 
        #for calculate sigmas
        if self.STORE_NUM < 100:
            self.store_randmark_pos.append(randmark_pos)

        elif self.STORE_NUM == 100:            
            for i in range(100):
                print(self.store_randmark_pos[i][0], ", ")
            for i in range(100):
                print(self.store_randmark_pos[i][1], ", ")
           
        
        self.STORE_NUM += 1
        """       

        # ---------- calculate pose for particle and randmark ---------------#
        for i in range(self.PARTICLES_NUM):
            true_randmark_pos = [0.0, 2.0, 0.0]
            dx = true_randmark_pos[0] - self.particles_x[i]
            dy = true_randmark_pos[1] - self.particles_y[i]
            d = math.sqrt(dx ** 2 + dy ** 2)
            phi = math.atan2(dy, dx) - self.particles_theta[i]
            while phi >= np.pi:
                phi -= 2 * np.pi
            while phi < -np.pi:
                phi += 2 * np.pi

            particle_suggest_pos = [d, phi]
            self.get_logger().info("l: %f" % particle_suggest_pos[0] + ", phi: %f" % particle_suggest_pos[1])

            ###尤度の計算###
            distance_dev_rate = 1000
            direction_dev = 1000
            obs_pos = randmark_pos
            distance_dev = distance_dev_rate * particle_suggest_pos[0]
            cov = np.diag(np.array([distance_dev ** 2, direction_dev ** 2]))
            self.weights[i] *= multivariate_normal(mean = particle_suggest_pos, cov = cov).pdf(obs_pos)
            print(self.weights[i])


        # ---------- particles publish------------- #
        # calc ideal pose with Dead Reckoning
        self.ideal_theta += self.omega * self.DELAT_T
        self.ideal_x += self.nu * self.DELAT_T * math.cos(self.ideal_theta)
        self.ideal_y += self.nu * self.DELAT_T * math.sin(self.ideal_theta)

        markerArray = MarkerArray()
        for i in range(self.PARTICLES_NUM):
            # add noise
            noise_rate_pdf = multivariate_normal(cov = self.C)
            ns = noise_rate_pdf.rvs()
            noised_nu = self.nu + ns[0] * math.sqrt(abs(self.nu) / self.DELAT_T) + ns[1] * math.sqrt(abs(self.omega) / self.DELAT_T)
            noised_omega = self.omega + ns[2] * math.sqrt(abs(self.nu) / self.DELAT_T) + ns[3] * math.sqrt(abs(self.omega) / self.DELAT_T)

            # calc particles pose with Dead Reckoning
            self.particles_theta[i] += noised_omega * self.DELAT_T
            self.particles_x[i] += noised_nu * self.DELAT_T * math.cos(self.particles_theta[i])
            self.particles_y[i] += noised_nu * self.DELAT_T * math.sin(self.particles_theta[i])

            # convert oira to quo
            rot = Rotation.from_rotvec(np.array([0, 0, self.particles_theta[i]]))
            quo = rot.as_quat()
            
            marker = Marker()
            marker.id = i
            marker.header.frame_id = "/odom"
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.scale.x = 0.5 * self.weights[i] * self.PARTICLES_NUM + 0.001
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.pose.position.x = self.particles_x[i]
            marker.pose.position.y = self.particles_y[i]
            marker.pose.position.z = 0.
            marker.pose.orientation.x = quo[0]
            marker.pose.orientation.y = quo[1]
            marker.pose.orientation.z = quo[2]
            marker.pose.orientation.w = quo[3]
            t = Duration()
            marker.lifetime = t            
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        
            markerArray.markers.append(marker)

        self.particles_pub.publish(markerArray)


def main(args=None):
    rclpy.init(args=args)
    particles_ctrl = ParticlesCtrl()
    rclpy.spin(particles_ctrl)
    particles_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

