🤖 Robot_gazebo_ws

This repository contains a ROS 2 Humble workspace with a differential drive robot simulated in Gazebo, designed for autonomous exploration, mapping, and navigation.

The main goal is to enable the robot to understand its environment, build a map with SLAM Toolbox, store it for future sessions, and later localize itself with AMCL within that map. Once localized, the robot can plan trajectories with Nav2 and avoid obstacles in real time, combining global path planning with local controllers.

This project integrates:

🌍 Gazebo simulation of multiple environments with obstacles.

🗺️ Mapping and saving maps using SLAM Toolbox.

📌 Accurate localization with AMCL on prebuilt maps.

🧭 Path planning and obstacle avoidance with Nav2.

🎮 Manual teleoperation with joystick or keyboard, plus autonomous navigation.

👨‍💻 Custom Python nodes for publishers and subscribers in ROS 2.

✅ System Requirements

🐧 Ubuntu 22.04 LTS

🤖 ROS 2 Humble Hawksbill

🦾 Gazebo (Fortress / Classic in Humble)

📦 Required ROS 2 packages:
