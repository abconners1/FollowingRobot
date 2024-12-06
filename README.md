# FollowingRobot

This repository implements a robot follower using ROS and PCL (Point Cloud Library). The robot computes the centroid and normals of detected points in a point cloud and generates velocity commands to follow a target. The project supports visualization using RViz and includes launch files for easier execution.

---

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [Dependencies](#dependencies)
- [Setup Instructions](#setup-instructions)
- [Usage](#usage)
  - [Running the Node](#running-the-node)
  - [Visualization in RViz](#visualization-in-rviz)
- [Files and Directories](#files-and-directories)
  - [Launch Files](#launch-files)
  - [RViz Configuration](#rviz-configuration)
  - [CMake Configuration](#cmake-configuration)
- [Contact](#contact)

---

## Overview

The **FollowingRobot** project is a ROS-based solution for creating a robot follower that:
- Processes depth data from a 3D sensor (e.g., depth camera or LiDAR).
- Computes the centroid of the detected object or region of interest (ROI).
- Publishes velocity commands for aligning and following the target.

---

## Features

- **Centroid Detection**: Computes the centroid from the point cloud.
- **Normal Estimation**: Calculates normals for better understanding of the environment (optional).
- **ROS Integration**: Fully integrated with ROS for subscribing to point clouds and publishing velocity commands.
- **RViz Visualization**: Visualize the processed data and robot commands in RViz.
- **Launch Files**: Simplified execution using pre-configured launch files.

---

## Dependencies

Ensure the following dependencies are installed:

- **ROS**: Noetic or Melodic
- **PCL**: Version compatible with your ROS distribution
- **RViz**: For visualization
- **CMake**: Version 3.5 or higher

Install missing packages using:
```bash
sudo apt-get install ros-<distro>-pcl-ros ros-<distro>-geometry-msgs ros-<distro>-sensor-msgs
