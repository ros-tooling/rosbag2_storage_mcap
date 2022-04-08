# rosbag2_storage_mcap

rosbag2 storage implementation for [MCAP](https://github.com/foxglove/mcap) file format

Packages in this repository:
* `rosbag2_storage_mcap`: a rosbag2 storage plugin that uses the C++ MCAP implementation to read and write MCAP files

## Usage

To build rosbag2_storage_mcap from source, in a shell that has your ROS2 environment sourced:

```bash
git clone https://github.com/ros-tooling/rosbag2_storage_mcap.git
cd rosbag2_storage_mcap
colcon build
source install/setup.bash
ros2 bag info -s mcap path/to/your_recording.mcap
```
