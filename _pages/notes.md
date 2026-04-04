---
title: "Notes"
permalink: /notes/
---

For the moment being, this page will host some code snippets that I'm tired of
searching for over and over again. I will try to keep it updated with the most
useful ones.

## Synonymous archive/backup files

Brace expansion can be used to quickly create archive or backup files synonymous
to the original file or directory.

- Tar

  ```bash
  tar -czvf archive{.tar.gz,}
  ```

- Zip
  ```bash
  zip -r backup{.zip,}
  ```

## Prevent building CMake tests/examples

When invoking cmake on the command line with an appropriate shell, you can use
brace expansion to quickly disable building tests, examples, and other optional
components at once.

The following command can be easily copied to blanket-disable building tests,
examples, benchmarks, and docs, accounting for some spelling variations:

```bash
cmake -S . -B build -D BUILD_{TESTS,TESTING,EXAMPLES,BENCHMARKS,DOCS}
```

## Running a GUI application in a Docker container

First, I need to allow the container to access my X server:

```bash
xhost +local:root
```

Then, I can run the container with the following command:

```bash
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v ~/.Xauthority:/root/.Xauthority \
  --device /dev/dri \
  --name my-gui-app \
  my-gui-app-image
```

As seen from `/root/.Xauthority`, this command only applies if the container is
run as root; Running containers as non-root users is another can of worms...

## Update alternatives

The right way to install a new alternative for a command is to use the
`update-alternatives --install` command. The role of the 4 confusingly
isomorphic parameters is as follows:

```bash
#                                  Symlink         Name Actual path    Priority
#                                  vvvvvvvvvvv     vvv  vvvvvvvvvvvvvv vvv
sudo update-alternatives --install /usr/bin/g++-13 g++ /usr/bin/g++-13 100
```

## Filtering out covariance data in ROS messages

In ROS 2, `ros2 topic echo` prints all fields of a message, including covariance
data in `geometry_msgs/PoseWithCovariance`, `nav_msgs/Odometry`, and other
messages.

The covariance data is printed as a 36-element array in yaml list format (i.e.,
one element per line). This swamps the terminal and obscures the usually more
interesting position and velocity fields.

### Attempted solutions:

- `--field`: This option only allows printing a single field, so it cannot be
  used to print all fields except covariance.

- `--filter`: This option only allows filtering messages based on field values,
  not dropping portions of the message.

### Actual solution:

Use `yq` to delete the covariance field from the message before printing it. For
example, to dropping covariances from the `pose` and `twist` fields
`nav_msgs/Odometry` messages, run:

```bash
ros2 topic echo /my_topic | yq 'del(.pose.covariance) | del(.twist.covariance)'
```

> `yq` is easily available on Ubuntu. It depends on `jq`, which is available via
> `sudo apt install jq`, and `yq` can be installed via `pipx install yq`; `pipx`
> itself can be installed via `sudo apt install pipx` followed by
> `pipx ensurepath`.
