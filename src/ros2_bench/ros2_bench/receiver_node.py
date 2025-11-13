import rclpy
from rclpy.node import Node
from custom_ping.msg import Ping
from ros2_bench.qos_profiles import QOS_PROFILES, on_deadline_missed, on_message_lost
from rclpy.event_handler import SubscriptionEventCallbacks
import sys
from ros2_bench.qos_profiles import QOS_PROFILES

class ReceiverNode(Node):
    def __init__(self, qos):
        super().__init__('receiver_node')
        event_callbacks = SubscriptionEventCallbacks(
            message_lost=lambda event: on_message_lost(event, self.get_logger()),
            deadline=lambda event: on_deadline_missed(event, self.get_logger())
        )

        self.received = []
        sender_topic = 'sender_topic'
        receiver_topic = 'receiver_topic'
        self.sub = self.create_subscription(Ping, sender_topic, self.on_ping_received, qos, event_callbacks=event_callbacks)
        self.pub = self.create_publisher(Ping, receiver_topic, qos)
        self.get_logger().info(f"Responder started. Listening on {sender_topic} and publishing on {receiver_topic}")


    def on_ping_received(self, msg):
        #Initialize a reply with the same contents as the received message
        reply = Ping()
        reply.seq = msg.seq
        reply.stamp = msg.stamp
        reply.payload = msg.payload
        reply.cpu_percent = msg.cpu_percent
        self.pub.publish(reply)
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