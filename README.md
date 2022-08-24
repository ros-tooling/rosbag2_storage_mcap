# rosbag2_storage_mcap

This package provides a [storage plugin](https://github.com/ros2/rosbag2#storage-format-plugin-architecture) for rosbag2 which extends it with support for the [MCAP](https://mcap.dev) file format.

[![ROS Foxy version](https://img.shields.io/ros/v/foxy/rosbag2_storage_mcap)](https://index.ros.org/p/rosbag2_storage_mcap/github-ros-tooling-rosbag2_storage_mcap/#foxy)
[![ROS Galactic version](https://img.shields.io/ros/v/galactic/rosbag2_storage_mcap)](https://index.ros.org/p/rosbag2_storage_mcap/github-ros-tooling-rosbag2_storage_mcap/#galactic)
[![ROS Humble version](https://img.shields.io/ros/v/humble/rosbag2_storage_mcap)](https://index.ros.org/p/rosbag2_storage_mcap/github-ros-tooling-rosbag2_storage_mcap/#humble)
[![ROS Rolling version](https://img.shields.io/ros/v/rolling/rosbag2_storage_mcap)](https://index.ros.org/p/rosbag2_storage_mcap/github-ros-tooling-rosbag2_storage_mcap/#rolling)


## Installation

rosbag2_storage_mcap is available as part of the [current ROS 2 distributions](https://docs.ros.org/en/rolling/Releases.html). On Ubuntu, after following the [ROS 2 installation instructions](https://docs.ros.org/en/humble/Installation.html), you can use:

```bash
# Replace "humble" with your ROS distro (`echo $ROS_DISTRO`)
$ sudo apt install ros-humble-rosbag2-storage-mcap
```

## Usage

Use MCAP files with regular [`ros2 bag` commands](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html) by adding the `--storage mcap` option (abbreviated as `-s mcap`):

```bash
$ ros2 bag record -s mcap /topic1 /topic2 ...

$ ros2 bag play -s mcap path/to/your_recording.mcap

$ ros2 bag info -s mcap path/to/your_recording.mcap
```


### Writer Configuration

To configure details of the MCAP writer for `ros2 bag record`, use the `--storage-config-file` options to provide a YAML file describing `mcap::McapWriterOptions`. See the following example providing some options.

See [McapWriterOptions declaration](https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/writer.hpp#L18) for all fields. Any omitted values will be left as default value.

NOTE: enums are accepted as a string matching the C++ declaration, rather than by underlying integer value.


```
# mcap_writer_options.yml
noCRC: false
noChunking: false
noMessageIndex: false
noSummary: false
chunkSize: 786432
compression: "Zstd"
compressionLevel: "Fast"
forceCompression: false
```

```
$ ros2 bag record -s mcap -o my_bag --all --storage-config-file mcap_writer_options.yml
```


## Development

To build `rosbag2_storage_mcap` from source:

```bash
$ source /opt/ros/[distro]/setup.bash
$ git clone https://github.com/ros-tooling/rosbag2_storage_mcap.git
$ cd rosbag2_storage_mcap
$ colcon build

# In another shell:
$ source /opt/ros/[distro]/setup.bash
$ cd rosbag2_storage_mcap
$ source install/local_setup.bash
$ ros2 bag info -s mcap path/to/your_recording.mcap
```
