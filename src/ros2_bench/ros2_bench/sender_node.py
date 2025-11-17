import rclpy
from rclpy.node import Node
import time, psutil, csv, os, datetime
from custom_ping.msg import Ping
from ros2_bench.qos_profiles import QOS_PROFILES, on_deadline_missed, on_message_lost
from rclpy.event_handler import SubscriptionEventCallbacks
import os
import numpy as np
from rclpy.serialization import serialize_message



class SenderNode(Node):
    def __init__(self):
        super().__init__('sender_node')
        self.log_path = os.path.expanduser("~/ros2_logs")
        os.makedirs(self.log_path, exist_ok=True)
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

        self.payload_size = 100 #In bytes
        self.pub = self.create_publisher(Ping,f"sender_topic_{self.topic_int}", qos_profile)
        self.sub = self.create_subscription(Ping, f"receiver_topic_{self.topic_int}", self.on_receive, qos_profile,event_callbacks=event_callbacks)
        self.seq = 0 # See how many messages are sent back
        self.max_msgs = 1500 # Change how many times the message is sent
        self.timer = self.create_timer(0.05, self.send_ping) # Change how fast the topic is sent
        
        #Initializing values to try and exit safely
        self.waiting = False
        self.wait_start_time = None
        self.last_reply_time = None
        self.wait_timeout = 8.0
        self.idle_timeout = 2.0
        self.wait_checker = self.create_timer(0.5, self.check_completion)


        self.latencies = []
        self.received = set()
        self.cpu_usage = []
        self.proc = psutil.Process(os.getpid())
        self.start_time = time.monotonic()
        self.curr_td = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_path, f"results_{self.topic_int}_{self.curr_td}")
        self.csv_file = open(f"{self.filename}.csv", "w", newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['seq', 'rtt_ms', 'cpu_percent'])
        self.payload = bytearray(np.random.randint(0,256,self.payload_size, dtype=np.uint8))
        self.out_of_order = 0
        self.duplicates = 0
        self.highest_seq_received = -1
        self.first_receive_time = None
        self.received_bytes = 0



    def send_ping(self):
        #Stops the alloted amount of messages has been sent
        if self.seq >= self.max_msgs:
            if not self.waiting:
                self.get_logger().info("All messages sent - waiting for responses")
                self.waiting = True
                self.wait_start_time = time.monotonic()
                try:
                    self.timer.cancel()
                except:
                    pass
            return
        
        msg = Ping()
        msg.seq = self.seq
        current_time = time.monotonic()
        msg.stamp.sec = int(current_time)
        msg.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
        cpu_percent = self.proc.cpu_percent(interval=None)
        self.cpu_usage.append(cpu_percent)
        msg.payload = self.payload
        msg.message = 'If you see this, its not encrypted'
        msg.cpu_percent = cpu_percent
        self.pub.publish(msg)
        self.seq += 1

    def on_receive(self, msg):
        #Time.monotonic instead of time.time() so ntp desync doesn't happen
        recv_time = time.monotonic()

        if self.first_receive_time is None:
            self.first_receive_time = recv_time

        sent_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        rtt_ms = (recv_time - sent_time) * 1000
        self.latencies.append(rtt_ms)

        #Get the time for the latest reply
        self.last_reply_time = time.monotonic()

        #Doesn't take into account DDS overhead
        self.received_bytes += len(serialize_message(msg))

        #Track packets seperately for each topic_int
        key = (self.topic_int, msg.seq)

        if key in self.received:
            self.duplicates += 1
        # Needs optimizing, maybe not writing after receive but all at once when finished.
        #Ignore duplicates if network isn't stable
        else:
            self.received.add(key)

            #Track out of order messages from the highest seq received
            if msg.seq < self.highest_seq_received:
                self.out_of_order += 1

            if msg.seq > self.highest_seq_received:
                self.highest_seq_received = msg.seq
            cpu_to_log = self.cpu_usage[-1] if self.cpu_usage else 0
            self.csv_writer.writerow([f"{msg.seq}", f"{rtt_ms:.3f}", f"{cpu_to_log:.2f}"])


    def calc_jitter(self):
        jitter = 0.0
        if len(self.latencies) >= 2:
            latencies = np.array(self.latencies)
            succ_diff = np.diff(latencies)
            jitter = np.mean(np.abs(succ_diff))
        return jitter
    
    def check_completion(self):
        if not self.waiting:
            return

        curr = time.monotonic()


        #Received everything
        if len(self.received) >= self.max_msgs:
            self.get_logger().info("All replies received - finishing..")
            self.finish()
            return
        
        #Wait if there are any messages still coming (not 100% it works)
        if self.last_reply_time and (curr - self.last_reply_time) > self.idle_timeout:
            self.get_logger().info("Idle timeout")
            self.finish()
            return
        
        #No replies in a certain amount of time so we're finishing
        if (curr - self.wait_start_time) > self.wait_timeout:
            self.get_logger().warn(f"Wait timeout {self.wait_timeout}s reached - finishing")
            self.finish()
            return

    def finish(self):
        if self.last_reply_time:
            elapsed = self.last_reply_time - self.start_time
        else:
            elapsed = time.monotonic() - self.start_time
        total = self.seq
        got = len(self.received)
        loss = total-got
        loss_rate = loss/total*100
        avg_rtt = sum(self.latencies)/len(self.latencies) if self.latencies else float('inf')
        avg_cpu = sum(self.cpu_usage)/len(self.cpu_usage) if self.cpu_usage else 0
        throughput_mbps = (self.received_bytes * 8) / elapsed / 1e6 if elapsed > 0 else 0
        summary = f"Benchmark summary for {self.qos_name} \n Messages sent: {total} \n Messages received {got} \n Total data received: {self.received_bytes/1e6:.2f} MB \n Packet loss: {loss_rate:.2f} % \n Avg RTT: {avg_rtt:.3f} ms \n Avg CPU: {avg_cpu:.2f} % \n Duration: {elapsed:.2f} s \n Jitter: {self.calc_jitter():.3f} ms \n Throughput: {throughput_mbps:.3f} mbps \n Out of order packets: {self.out_of_order} \n Duplicate packets: {self.duplicates} \n"
        self.get_logger().info(summary)
        self.csv_file.close()
        with open(f"{self.filename}.txt", "w") as f:
            f.write(summary)
        
        self.destroy_node()
        rclpy.shutdown()


def main():
    print("Initializing node")
    rclpy.init()
    print(f"Setting up node")
    node = SenderNode()
    print(node.qos_name)
    rclpy.spin(node)
    print("Shutting down")
    
    

if __name__ == '__main__':
    main()


