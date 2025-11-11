# ros2-benchmark
Simple benchmark to test the latency between two python nodes using FastDDS


## Usage

```bash
clone the repository: git clone https://github.com/dashecc/ros2-benchmark.git
colcon build
source install/setup.bash

Run with:
ros2 run ros2_bench sender_node <QoS Profile> for example best_effort_volatile

Repeat for receiver, but it should be moved to another system unless shared memory is disabled in FastDDS
```
