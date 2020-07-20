import rclpy
from visualization_msgs.msg import Marker
from rclpy.node import Node


class MyMarker(Node):
    
    TOPIC = 'maker_pub'

    def __init__(self):

        super().__init__(self.TOPIC)

        self.get_logger().info("%s initializing..." % (self.TOPIC))

        self.pub = self.create_publisher(Marker, self.TOPIC, 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rclpy.time

        marker_data.ns = "basic_shapes"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = 0.0
        marker_data.pose.position.y = 0.0
        marker_data.pose.position.z = 0.0

        marker_data.pose.orientation.x=0.0
        marker_data.pose.orientation.y=0.0
        marker_data.pose.orientation.z=1.0
        marker_data.pose.orientation.w=0.0

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 1.
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        #marker_data.lifetime = rclpy.Duration

        marker_data.type = 0
    
        self.pub.publish(marker_data)
        self.get_logger().info('Publishing: "%s"' % marker_data)



def main(args=None):
    
    rclpy.init(args=args)

    my_marker = MyMarker()

    rclpy.spin(my_marker)

    my_marker.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()



