import rclpy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from rclpy.node import Node

from builtin_interfaces.msg import Duration


class MyMarker(Node):
    
    TOPIC = 'maker_pub'

    def __init__(self):

        super().__init__(self.TOPIC)

        self.get_logger().info("%s initializing..." % (self.TOPIC))

        self.pub = self.create_publisher(MarkerArray, self.TOPIC, 10)

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        marker_data = Marker()
        marker_array = MarkerArray()

        for i in range(5):
            marker_data.header.frame_id = "map"
            #marker_data.header.stamp = rclpy.time

            marker_data.ns = "basic_shapes"
            marker_data.id = i

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = 0.0
            marker_data.pose.position.y = 0.0
            marker_data.pose.position.z = float(i)

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

            marker_data.lifetime = Duration()

            marker_data.type = 0

            marker_array.markers.append(marker_data)
    
            self.pub.publish(marker_array)

        self.get_logger().info('Publishing: "%s"' % marker_array.markers[0].pose)



def main(args=None):
    
    rclpy.init(args=args)

    my_marker = MyMarker()

    rclpy.spin(my_marker)

    my_marker.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()



