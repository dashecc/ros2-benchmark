import rclpy
from rclpy.node import Node
import time, psutil, csv, os, datetime, sys
from custom_ping.msg import Ping
from ros2_bench.qos_profiles import QOS_PROFILES, on_deadline_missed, on_message_lost
from rclpy.event_handler import SubscriptionEventCallbacks
import os
import numpy as np



class SenderNode(Node):
    def __init__(self, qos_profile, name):
        super().__init__('sender_node')
        self.log_path = os.path.expanduser("~/ros2_logs")
        os.makedirs(self.log_path, exist_ok=True)
        event_callbacks = SubscriptionEventCallbacks(
            message_lost=lambda event: on_message_lost(event, self.get_logger()),
            deadline=lambda event: on_deadline_missed(event, self.get_logger())
        )
        self.payload_size = 500000 #Simulate a large payload
        self.name = name
        self.pub = self.create_publisher(Ping,"sender_topic", qos_profile)
        self.sub = self.create_subscription(Ping, "receiver_topic", self.on_receive, qos_profile,event_callbacks=event_callbacks)
        self.seq = 0 # See how many messages are sent back
        self.max_msgs = 500 # Change how many times the message is sent
        self.latencies = []
        self.received = set()
        self.cpu_usage = []
        self.proc = psutil.Process(os.getpid())
        self.start_time = time.time()
        self.timer = self.create_timer(0.04, self.send_ping) # Change how fast the topic is sent
        self.curr_td = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_path, f"results_{self.name}_{self.curr_td}")
        self.csv_file = open(f"{self.filename}.csv", "w", newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['seq', 'rtt_ms', 'cpu_percent'])
        self.payload = bytearray(np.random.randint(0,256,self.payload_size, dtype=np.uint8))



    def send_ping(self):
        #Stops the alloted amount of messages has been sent
        if self.seq >= self.max_msgs:
            self.finish()
            return
        
        msg = Ping()
        msg.seq = self.seq
        current_time = time.monotonic()
        msg.stamp.sec = int(current_time)
        msg.stamp.nanosec = int((current_time - int(current_time)) * 1e9)
        cpu_percent = self.proc.cpu_percent(interval=None)
        self.cpu_usage.append(cpu_percent)
        msg.payload = self.payload
        msg.cpu_percent = cpu_percent
        self.pub.publish(msg)
        self.seq += 1

    def on_receive(self, msg):
        #Time.monotonic instead of time.time() so ntp desync doesn't happen
        recv_time = time.monotonic()
        sent_time = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        rtt_ms = (recv_time - sent_time) * 1000
        self.latencies.append(rtt_ms)

        #Ignore duplicates if network isn't stable
        if msg.seq not in self.received:
            self.received.add(msg.seq)
            cpu_to_log = self.cpu_usage[-1] if self.cpu_usage else 0
            self.csv_writer.writerow([f"{msg.seq}", f"{rtt_ms:.3f}", f"{cpu_to_log:.2f}"])


    def calc_jitter(self):
        jitter = 0.0
        if len(self.latencies) >= 2:
            latencies = np.array(self.latencies)
            succ_diff = np.diff(latencies)
            jitter = np.mean(np.abs(succ_diff))
        return jitter

    def finish(self):


        total = self.seq
        got = len(self.received)
        loss = total-got
        loss_rate = loss/total*100
        avg_rtt = sum(self.latencies)/len(self.latencies) if self.latencies else float('inf')
        avg_cpu = sum(self.cpu_usage)/len(self.cpu_usage) if self.cpu_usage else 0
        summary = f"Benchmark summary for {self.name} \n Messages sent: {total} \n Messages received {got} \n Packet loss: {loss_rate:.2f} % \n Avg RTT: {avg_rtt} ms \n Avg CPU: {avg_cpu:.2f} % \n Duration: {time.time()-self.start_time:.2f} s \n Jitter: {self.calc_jitter()} \n"
        self.get_logger().info(summary)
        self.csv_file.close()
        with open(f"{self.filename}.txt", "w") as f:
            f.write(summary)
        rclpy.shutdown()


def main():
    if len(sys.argv) < 2:
        print("Usage: ros2 run ros2_bench sender_node <qos_profile> e.g best_effort_volatile")
    
    name = sys.argv[1]

    #Look for the name given in the QOS_PROFILES dictionary and set that as the qos profile for the run
    qos = QOS_PROFILES[name]
    print("Initializing node")
    rclpy.init()
    print(f"Setting up node {name}")
    node = SenderNode(qos, name)
    rclpy.spin(node)
    print("Shutting down")
    
    

if __name__ == '__main__':
    main()


