# ros2-benchmark

Simple benchmarking tool for ROS 2, allowing 6 different QoS profiles to be tested simultaneously, optionally with SROS2 security enabled.

## Features

- Add/use preconfigured QoS profiles to test the network.
- Launch multiple iterations of the same node to create realistic ROS 2 traffic.
- Optional SROS2 security testing.
- Single launch file works for both secure and non-secure testing.
- Payload size and message interval can be easily configured for custom benchmarks.

## Requirements

- Ubuntu 24.04 (Jazzy)
- FastDDS middleware (tested)
- ROS 2 Jazzy installed

## Installation

1. Clone the repository:

```bash
git clone https://github.com/dashecc/ros2-benchmark.git
cd ros2-benchmark
```

2. Build the packages:

```bash
colcon build --packages-select custom_ping ros2_bench
source install/setup.bash
```

> Note: You can clone this repository anywhere. As long as you build and source the workspace, ROS 2 commands will work.

## Usage

### Setting Custom Values

You can modify these directly in `sender_node.py`:

- **Payload size**: `self.payload_size` (default 5000 bytes)
- **Message interval**: `self.timer = self.create_timer(interval, self.send_ping)` (default 0.05 s)

### Non-Secure Benchmarking

Run sender and receiver nodes without security:

```bash
ros2 launch launch/sender.launch.py
ros2 launch launch/receiver.launch.py
```

### Secure Benchmarking (Optional)

To run benchmarks with SROS2 security:

1. Enable ROS 2 security and set the keystore path:

```bash
export ROS_SECURITY_KEYSTORE=~/<PATH_TO_KEYSTORE>
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

2. Launch nodes with the security enclave:

```bash
ros2 launch launch/sender.launch.py enclave:=/talker_listener/talker
ros2 launch launch/receiver.launch.py enclave:=/talker_listener/listener
```

**Notes:**

- The `enclave` argument corresponds to a subfolder in your SROS2 keystore.
- Can use the same enclave for every node created

## Launch File Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| **enclave** | string | `''` | Optional SROS2 enclave path. Only used if security is enabled. |

## Node Naming Convention

Nodes are automatically named in a loop:

```text
sender_node_1 - reply_node_1
sender_node_2 - reply_node_2
sender_node_3 - reply_node_3
...
sender_node_6 - reply_node_6
```

Each node can use the same keystore folder if desired.

## QoS Profile Mapping

The following mapping is consistent throughout the project:

1. reliable_volatile  
2. best_effort_volatile  
3. reliable_transient_local  
4. best_effort_transient_local  
5. reliable_keep_all  
6. best_effort_keep_all

## Log Creation and Plotting

- Summary `.txt` and `results.csv` files are written to `~/ros2_logs/`.  
- CSV files can be plotted using `graph.py` in `/utils`. The script automatically detects CSV files and generates graphs.

## Custom Messages

`custom_ping.msg` contains:

- `uint64 seq`  
- `builtin_interfaces/Time stamp`  
- `float64 cpu_percent`  
- `uint8[] payload` (customizable size)  
- `string message` (optional message content)  

Use these values to control what is sent.

## Metrics Calculated

- **Messages sent**: Total messages published  
- **Messages received**: Total unique messages received  
- **Packet loss (%)**: Percentage of messages not received  
- **Round-trip time (RTT, ms)**: Average time from sending a message to receiving its reply  
- **Jitter (ms)**: Average absolute difference between consecutive RTTs  
- **CPU usage (%)**: Average CPU usage of the sending node during the test  
- **Throughput (Mbps)**: Total data received per second  
- **Out-of-order packets**: Number of messages arriving with a sequence number lower than the highest received  
- **Duplicate messages**: Number of messages received more than once

## Notes / Performance Considerations

- RTT is measured with `time.monotonic()` for consistency across nodes and machines.  
- Throughput is predictable because messages are sent at fixed intervals.  
- Throughput does not include encryption overhead.  
- Packets can be dropped in two places: sender → receiver or receiver → sender. Only successful replies are counted as received.  
- By default, FastDDS uses **shared memory (shm)** for local communication. For realistic network testing, run the receiver node on a separate machine.

## Example Quickstart

**Non-secure:**

```bash
ros2 launch launch/sender.launch.py
ros2 launch launch/receiver.launch.py
```

**Secure:**

```bash
export ROS_SECURITY_KEYSTORE=~/sros2/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce

ros2 launch launch/sender.launch.py enclave:=/talker_listener/talker
ros2 launch launch/receiver.launch.py enclave:=/talker_listener/listener
```

```
```

