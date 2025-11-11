import rclpy
from rclpy.node import Node
from custom_ping.msg import Ping
import sys
from ros2_bench.qos_profiles import QOS_PROFILES

class ReceiverNode(Node):
    def __init__(self, qos):
        super().__init__('receiver_node')

        self.received = []
        sender_topic = 'sender_topic'
        receiver_topic = 'receiver_topic'
        self.sub = self.create_subscription(Ping, sender_topic, self.on_ping_received, qos)
        self.pub = self.create_publisher(Ping, receiver_topic, qos)
        self.get_logger().info(f"Responder started. Listening on {sender_topic} and publishing on {receiver_topic}")


    def on_ping_received(self, msg):
        #Bounce back the message once it has been received
        self.pub.publish(msg)
        self.received.append(msg.seq)
        self.get_logger().debug(f"Bounced message with seq: {msg.seq}")


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 2:
        print("ros2 run ros2_bench <qos_mode> for example best_effort_volatile")
    
    name = sys.argv[1]

    #Look for the name given in the QOS_PROFILES dictionary and set that as the qos profile for the run
    qos = QOS_PROFILES[name]


    responder = ReceiverNode(qos)
    try:
        rclpy.spin(responder)
    except:
        KeyboardInterrupt
        pass
    
    print(responder.received)
    responder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()