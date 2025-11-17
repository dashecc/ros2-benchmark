# ros2-benchmark
Simple benchmarking tool, allowing 6 different QoS profiles to be tested simulatenously, optionally with SROS2 enabled.

## Features
- Add/Use preconfigured QoS profiles to test the network
- Launch multiple iterations of the same node to create realistic ROS 2 traffic on the network
- Optional testing with SROS2
- Single launch file that works both for security and non-security testing
- Payload size and different settings can easily be configured for any kind of testing

## Requirements
- Only tested on Ubuntu 24.04 running Jazzy
- Only tested with FastDDS middleware



## Installation

1. Clone the repository
```bash
Clone the repository: git clone https://github.com/dashecc/ros2-benchmark.git
Change to the directory: cd ros2-benchmark
```
2. Build the packages
```bash
Build the workspace: colcon build or colcon build --packages-select custom_ping ros2_bench
Source the workspace: source install/setup.bash
```

## Usage

### Setting custom values

1. Control the size of the payload
```bash
line 32 sender_node.py: self.payload_size = <set integer value> the value is seen as bytes.
```
2. Control how fast messages are sent
```bash
line 37 sender_node.py: self.timer = self.create_timer(0.05, self.send_ping)
```

### Non-secure benchmarking

1. Start sender/receiver without security
```bash
ros2 launch launch/sender.launch.py
ros2 launch launch/receiver.launch.py
```

### Secure testing (Optional)

To run the benchmark with security enabled, you need to specify an **enclave**

1. Enable ROS 2 security
```bash
export ROS_SECURITY_KEYSTORE=~/<PATH_TO_KEYSORE>
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```
2. Launch nodes with the security enclave

Replace the enclave argument with the path to the enclave in the keystore you created.
Using the enclave path from ROS 2 Jazzy docs: Setting up security as an example of a valid argument

```bash
ros2 launch launch/sender.launch.py enclave:=/talker_listener/talker < REPLACE
ros2 launch launch/receiver.launch.py enclave:=/talker_listener/listener < REPLACE
```

## Launch File Parameters
| Parameter | Type | Default | Description |
| -------| -------| -------| ---------|
| **enclave** | string | '' | Optional SROS2 enclave path. Only used if security enabled |


## Node Naming Convention

Nodes are automatically named in a loop

```bash
sender_node_1 - reply_node_1
sender_node_2 - reply_node_2
sender_node_3 - reply_node_3
...
sender_node_6 - reply_node_6
```

Each node can use the same keystore that was generated

## QoS Profile Mapping:

This mapping is used throughout the project from initializing the nodes, for storing in the csv names and getting the QoS name for the graphs. This mapping stays consistant throughout the entire project

1. reliable_volatile
2.    best_effort_volatile
3.    reliable_transient_local
4.    best_effort_transient_local
5.    reliable_keep_all
6.    best_effort_keep_all

## Log Creation and Plotting

The .txt summary and results.csv are written into ~/ros2_logs/

The .csv files can be plotted with graph.py found in /utils, it is designed to automatically find the csv's and plot them.

## Custom Messages
- custom_ping.msg includes:
    - uint64 seq
    - builtin_interfaces/Time stamp
    - float64 cpu_percent
    - uint8[] payload - can customize the size
    - string message - can customize
- Use these values to control what is sent

## Metric Calculated

These are the following metrics that the tool provides:
- **Messages sent**: Total number of messages published.
- **Messages received**: Total unique messages received.
- **Packet loss (%)**: Percentage of messages that were not received.
- **Round-trip time (RTT, ms)**: Average time from sending the message to receiving its reply.
- **Jitter (ms)**: Average absolute difference between consecutive RTTs.
- **CPU usage (%)**: Average CPU usage of the sending node during the test.
- **Throughput (mbps)**: otal data received per second.
- **Out-of-order packets**: Number of received packets that arrived with a sequence number lower than the highest received.
- **Duplicate messages**: Number of messages received more than once.

## Notes

- RTT is measured with time.monotic() for consistency across nodes and machines
- Throughput is currently predictable due to the nature of sending x messages in x time
- Throughput doesn't take encryption overhead into consideration
- Due to the RTT nature, the packet can be dropped in 2 different places sender -> receiver or receiver -> sender, only successful replies are counted as received.