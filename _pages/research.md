---
title: "Research"
permalink: /research/
---

This page summarizes the timeline of my research work during my PhD, organized
by topic.

## Slung Load Transport

When I joined the FSC Lab in 2019 as a M.A.Sc, student, I worked on the control
of a quadrotor slung load transport system, under the mentorship of
[Dr. Longhao Qian](https://scholar.google.com/citations?user=p2oGLVYAAAAJ&hl=en),
as part of a collaboration project with Drone Delivery Canada.

In this stage, I built a fleet of S500 quadrotor experimental platforms and
interfaced them with the Pixhawk 4 autopilot and Jetson Nano companion computer.

I developed automation and orchestration scripts to help with experiment
management. I also developed a
[data collection software](https://github.com/FSC-Lab/ros_logger_gui) to
facilitate data collection and analysis for the project. [^1]

I paused my research work in 2020 due to COVID-19. After I returned to UTIAS in
2021, I moved on to implementing Longhao's slung load control algorithm for
outdoor flight. I achieved slung load transport with a single drone in outdoor
environments under high wind conditions in the Canadian November. The next May,
I demonstrated multi-drone slung load transport to our collaborators at Drone
Delivery Canada.

[^1]:
    This software has gone unmaintained since the transition to ROS 2, but I
    believe a graphical tool to configure ROS bag recording (Writing
    `ros2 bag record ...` once is too many) is still an open problem in the ROS
    ecosystem.

## Cooperative Localization

## Observability-Aware Control
