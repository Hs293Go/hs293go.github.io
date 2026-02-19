---
title: "Footguns"
permalink: /footguns/
---

```python
self.shoot(self.foot["left"])  # kaboom
```

This page is a collection of footguns that I have encountered.

## Easily confused software/package names

### [Rerun](https://rerun.io/)

The visualization framework's Python package is called `rerun-sdk`.

`rerun` is an unrelated CLI utility to literally _rerun_ files.

### [Just](https://github.com/casey/just)

The rust-based command runner's python package is called `rust-just`.

`just` is an unrelated library providing a terser syntax to read/write files.

## ROS2 jazzy/humble message incompatibility

ROS2 messages built under the _jazzy_ distribution (Ubuntu 24.04) are likely
**NOT** compatible with those built under the _humble_ (Ubuntu 22.04)
distribution. User defined messages are more likely to be incompatible.

### Example

On 2026-02-18, when I attempted to send user-defined messages from a NVIDIA
Jetson Orin running ROS _humble_ to a laptop running ROS _jazzy_ is:

```
Fast CDR exception deserializing message of type rms_dds_common::msg::dds_::ParticipantEntitiesInfo_, at <file>:<line>
```

When the error occurs, messages _appear_ to be sent and received, i.e.,
`ros2 topic echo` on the subscriber side shows messages being received, but the
data is silently corrupted between the publisher and subscriber, resulting in
zeroed-out fields in the received messages.

### Solution

No workarounds. Do not mix ROS2 distributions in the same system.

## ROS2 jazzy/humble rosbag incompatibility

ROS2 bags recorded under the _jazzy_ distribution (Ubuntu 24.04) are **NOT**
compatible with those recorded under the _humble_ (Ubuntu 22.04) distribution.

Bags recorded under the _jazzy_ distribution are in the `.mcap` format, while
those recorded under the _humble_ distribution are in the `.db3` format.

Although `ros2 bag record` under _jazzy_ offers a `--storage` option to specify
the storage format, the `metadata.yaml` file in the recorded bag have different
formats between the two distributions, and there appears to be no standard tool
to convert between the two formats.

### Solution

None. Since humble messages are incompatible with jazzy messages, you **CANNOT**
play humble messages in a `ros/humble` docker container and use them on the host
running jazzy, and vice versa.

## PX4 uses special QoS settings for ROS2 messages

The
[PX4 Autopilot](https://docs.px4.io/main/en/ros2/user_guide#ros-2-subscriber-qos-settings)
uses a special set of quality-of-service QoS settings when publishing its ROS2
messages. Failing to match these QoS settings on the subscriber side may result
in messages not being received.

### Solution

In C++, specify the QoS settings as follows:

```cpp
const auto qos_profile = rmw_qos_profile_sensor_data;
auto qos =
    rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
```

In Python, specify the QoS settings as follows:

```python
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

qos_profile = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
```

## PX4's system installer script breaks system packages

`pip` on Ubuntu 24.04 adopted [PEP 668](https://peps.python.org/pep-0668/) and
refuses to install into the system `site-packages` directory.[^1]

[^1]:
    This is a massively helpful feature to prevent breakage of system packages
    by `pip` installations, and arguably mitigates the problem of a "god
    environment" containing numerous, potentially conflicting packages installed
    by the user and the system.

The [PX4 Autopilot](https://docs.px4.io)'s system installer script
[ubuntu.sh](https://github.com/PX4/PX4-Autopilot/blob/main/Tools/setup/ubuntu.sh#L115),
however, explicitly passes `--break-system-packages` to `pip`, installing
packages into your system Python environment unless you run it in a virtual
environment,
[which is not mentioned in the installation instructions.](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu)

While the "break system packages" approach is practical for helping new users to
build PX4 and flash firmwares ASAP, it is not ideal for users who have other
Python projects and packages.

### Solution

Since PX4 does not have a Python API, it it perfectly fine to install the
required Python packages in a virtual environment. I demonstrate it with `uv`
here, but you can use any virtual environment tool of your choice.

```bash
cd PX4-Autopilot
uv venv
source .venv/bin/activate
uv pip install -r Tools/setup/requirements.txt

make px4_sitl # Example command to build a PX4 target
```

where the path to `requirements.txt` is relative to the root of the
PX4-Autopilot repository.
