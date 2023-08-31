# wros2

ROS2 node examples with WRS.

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/onrobot)

- ROS Humble Hawksbill node examples with robot motion planners implemented in [WRS](https://github.com/wanweiwei07/wrs).

# Features

- Docker environment for ROS Humble Hawksbill packages
- ROS2 node examples with a [grasp planner (Wan et al., IROS'17)](https://ieeexplore.ieee.org/abstract/document/8206011)

# Dependency

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - [ROS2 Humble Hawksbill](https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html)
  - docker 20.10.12
  - docker-compose 1.29.2
  - nvidia-docker2 2.8.0-1

# Installation
```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/wros2.git --recursive --depth 1
sudo apt install byobu -y
cd wros2
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

# Usage
```bash
docker compose up
```
```bash
xhost +
docker exec -it wros_humble_container bash
ros2 launch wros2_tutorials plan_grasp_launch.py
ros2 service call /plan_grasp std_srvs/srv/Empty
```

<img src=image/plan_grasp.gif width=720>  

# Contributors



# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  
