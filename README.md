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

To configure details of the MCAP writer for `ros2 bag record`, use the `--storage-config-file` options to provide a YAML file describing `mcap::McapWriterOptions`. Field descriptions below copied from [McapWriterOptions declaration](https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/writer.hpp#L18)

| Field | Type / Values | Description |
| ----- | ------------- | ----------- |
| noCRC | bool | Disable CRC calculations for Chunks, Attachments, and the Data and Summary sections. |
| noChunking | bool | Do write Chunks to the file, instead writing Schema, Channel, and Message records directly into the Data section. |
| noMessageIndex | bool | Do not write Message Index records to the file. If `noSummary=true` and `noChunkIndex=false`, Chunk Index records will still be written to the Summary section, providing a coarse message index. |
| noSummary | bool | Do not write Summary or Summary Offset sections to the file, placing the Footer record immediately after DataEnd. This can provide some speed boost to file writing and produce smaller files, at the expense of requiring a conversion process later if fast summarization or indexed access is desired. |
| chunkSize | unsigned int | Target uncompressed Chunk payload size in bytes. Once a Chunk's uncompressed data meets or exceeds this size, the Chunk will be compressed (if compression is enabled) and written to disk. Note that smaller Chunks may be written, such as the last Chunk in the Data section. This option is ignored if `noChunking=true`. |
| compression | "None", "Lz4", "Zstd" | Compression algorithm to use when writing Chunks. This option is ignored if `noChunking=true`. |
| compressionLevel | "Fastest", "Fast", "Default", "Slow", "Slowest" | Compression level to use when writing Chunks. Slower generally produces smaller files, at the expense of more CPU time. These levels map to different internal settings for each compression algorithm. |
| forceCompression | bool | By default, Chunks that do not benefit from compression will be written uncompressed. This option can be used to force compression on all Chunks. This option is ignored if `noChunking=true`. |
| noRepeatedSchemas | bool | Advanced option. |
| noRepeatedChannels | bool | Advanced option. |
| noMetadataIndex | bool | Advanced option. |
| noChunkIndex | bool | Advanced option. |
| noStatistics | bool | Advanced option. |
| noSummaryOffsets | bool | Advanced option. |


Example:

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

### ROS 2 Distro maintenance

Whenever a ROS 2 distribution reaches EOL, search for comments marked COMPATIBILITY - which may no longer be needed when no new releases will be made for that distro.
