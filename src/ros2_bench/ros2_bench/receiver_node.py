import rclpy
from rclpy.node import Node
from custom_ping.msg import Ping
from ros2_bench.qos_profiles import QOS_PROFILES, on_deadline_missed, on_message_lost
from rclpy.event_handler import SubscriptionEventCallbacks
import sys
from ros2_bench.qos_profiles import QOS_PROFILES

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')
        event_callbacks = SubscriptionEventCallbacks(
            message_lost=lambda event: on_message_lost(event, self.get_logger()),
            deadline=lambda event: on_deadline_missed(event, self.get_logger())
        )

        # Using a parameter is infinitely better than using a sys arg, can probably be condensed a little
        self.declare_parameter('qos_profile', 'reliable_volatile')
        qos_name = self.get_parameter('qos_profile')
        self.qos_name = qos_name.get_parameter_value().string_value
        qos_profile = QOS_PROFILES[self.qos_name]

        self.declare_parameter('topic_int',1)
        topic_int = self.get_parameter('topic_int')
        self.topic_int = topic_int.get_parameter_value().integer_value

        self.received = []
        sender_topic = f'sender_topic_{self.topic_int}'
        receiver_topic = f'receiver_topic_{self.topic_int}'
        self.sub = self.create_subscription(Ping, sender_topic, self.on_ping_received, qos_profile, event_callbacks=event_callbacks)
        self.pub = self.create_publisher(Ping, receiver_topic, qos_profile)
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
    responder = ReceiverNode()
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