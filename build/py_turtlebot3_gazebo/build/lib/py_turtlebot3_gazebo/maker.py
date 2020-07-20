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
        msg = Marker()
        msg.header.frame_id = "test test"        
        self.pub.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.header.frame_id)



def main(args=None):
    
    rclpy.init(args=args)

    my_marker = MyMarker()

    rclpy.spin(my_marker)

    my_marker.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()



