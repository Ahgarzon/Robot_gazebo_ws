# 🤖 Robot\_gazebo\_ws

This repository contains a **ROS 2 Humble workspace** with a **differential drive robot simulated in Gazebo**, designed for autonomous exploration, mapping, and navigation.

The main goal is to **enable the robot to understand its environment**, build a **map with SLAM Toolbox**, **store it for future sessions**, and later **localize itself with AMCL** within that map. Once localized, the robot can **plan trajectories with Nav2** and **avoid obstacles in real time**, combining global path planning with local controllers.

![Uploading image.png…]()


This project integrates:

* 🌍 **Gazebo simulation** of multiple environments with obstacles.
* 🗺️ **Mapping and saving maps** using **SLAM Toolbox**.
* 📌 **Accurate localization** with **AMCL** on prebuilt maps.
* 🧭 **Path planning and obstacle avoidance** with **Nav2**.
* 🎮 **Manual teleoperation** with joystick or keyboard, plus autonomous navigation.
* 👨‍💻 **Custom Python nodes** for publishers and subscribers in ROS 2.

---

## ✅ System Requirements

* 🐧 **Ubuntu 22.04 LTS**
* 🤖 **ROS 2 Humble Hawksbill**
* 🦾 **Gazebo (Fortress / Classic in Humble)**
* 📦 Required ROS 2 packages:

```bash
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-slam-toolbox \
                 ros-humble-teleop-twist-joy \
                 ros-humble-joy \
                 ros-humble-xacro
```

---

## 📥 Clone the Repository

```bash
cd ~
git clone https://github.com/Ahgarzon/Robot_gazebo_ws.git
cd Robot_gazebo_ws
```

---

## 🏗 Build the Workspace

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ How to Run

### 1️⃣ Launch Gazebo

```bash
ros2 launch modelo_robot gazebo.launch.py
```

### 2️⃣ View in RViz2

```bash
ros2 launch modelo_robot display.launch.py
```

### 3️⃣ Run SLAM

```bash
ros2 launch modelo_robot slam_nav2.launch.py
```

### 4️⃣ Localization with AMCL

```bash
ros2 launch modelo_robot nav_full.launch.py
```

### 5️⃣ Teleoperation

```bash
ros2 run teleop_twist_joy teleop_node
```

---

## 📂 Repository Structure

```
Robot_gazebo_ws/
├── src/
│   ├── modelo_robot/        # Main robot package
│   └── publicador_python/   # Custom Python nodes
├── launch.sh                # Helper launch script
├── my_map.yaml / .pgm       # Saved maps
└── rviz_2.rviz              # RViz2 configuration
```

---

## 🛠️ Tools

* **ROS 2 Humble**
* **Gazebo**
* **RViz2**
* **SLAM Toolbox**
* **Nav2**
* **AMCL**
* **Python (rclpy)**

---

## 🌐 Notes

* Make sure to configure *automatic sourcing* so ROS 2 and the workspace work in new terminals:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/Robot_gazebo_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

* SLAM-generated maps are saved under:
  `src/modelo_robot/maps/`

* Navigation parameters can be tuned in:
  `src/modelo_robot/config/nav2_params.yaml`

---

## 📊 Expected Results

* Differential drive robot able to **explore, map, and save environments**.
* **Accurate localization** using AMCL.
* **Autonomous navigation** with Nav2.
* **Real-time obstacle avoidance**.
* **Switch between manual teleoperation and autonomous mode**.
