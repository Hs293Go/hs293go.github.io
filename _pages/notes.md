---
title: "Notes"
permalink: /notes/
---

For the moment being, this page will host some code snippets that I'm tired of
searching for over and over again. I will try to keep it updated with the most
useful ones.

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
