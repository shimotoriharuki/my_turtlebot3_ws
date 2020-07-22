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

class ParticlesCtrl(Node): 

    def __init__(self):
        # variable define
        self.topic_vel = "cmd_vel"

        # pub sub initiarize
        super().__init__(self.topic_vel)
        self.get_logger().info("%s initializing..." % (self.topic_vel))

        self.vel_sub = self.create_subscription(Twist, self.topic_vel, self.vel_callback, 10)

    def vel_callback(self, msg):
        self.nu = msg.linear.x
        self.omega = msg.angular.z

        #self.get_logger().info("cmd_vel %s" % msg)
        self.get_logger().info("nu: %f" % self.nu + ", omega: %f" % self.omega)

def main(args=None):
    rclpy.init(args=args)
    particles_ctrl = ParticlesCtrl()
    rclpy.spin(particles_ctrl)
    particles_ctrl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

