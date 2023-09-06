# wros2

ROS2 node examples with WRS.

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/onrobot)

- ROS Humble Hawksbill node examples with robot motion planners implemented in [WRS](https://github.com/wanweiwei07/wrs).

# Features

- Docker environment for ROS Humble Hawksbill packages
- ROS2 node examples with [grasp planners (Wan et al., IEEE TRO 2021)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9170578)

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
1. Make and execute the docker environment
- Run the following command in one terminal
  ```bash
  docker compose up
  ```
- Run these commands in another terminal
  ```bash
  xhost +
  docker exec -it wros_humble_container bash
  ```
2. Revise the planning parameters described in [wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml)  
3. Build the source codes with the revised yaml file
    ```bash
    colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
    ```
4. Execute commands in the docker container  
    ```bash
    byobu
    ros2 launch wros2_tutorials plan_grasp_launch.py
    # create new window by clicking F2 key
    ros2 service call /plan_grasp std_srvs/srv/Empty
    ```

## [Robotiq Hand-E](https://robotiq.com/products/hand-e-adaptive-robot-gripper)
[wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml) 
```yaml
gripper_name: 'robotiqhe'
object_mesh_path: '/ros2_ws/src/wrs/0000_examples/objects/tubebig.stl'
```
<img src=image/robotiqhe.gif width=720>  

## [Robotiq 2F-85](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
[wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml) 
```yaml
gripper_name: 'robotiq85'
object_mesh_path: '/ros2_ws/src/wrs/0000_examples/objects/bunnysim.stl'
```
<img src=image/robotiq85.gif width=720>  

## [Robotiq 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
[wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml) 
```yaml
gripper_name: 'robotiq140'
object_mesh_path: '/ros2_ws/src/wrs/0000_examples/objects/milkcarton.stl'
```
<img src=image/robotiq140.gif width=720>  

## Suction gripper
[wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml) 
```yaml
gripper_name: 'suction'
object_mesh_path: '/ros2_ws/src/wrs/pyhiro/suction/objects/sandpart2.stl'
```
<img src=image/suction.gif width=720>  

## [CONVUM balloon hand SGB30](https://convum.co.jp/products/en/other-en/sgb/)
[wros2_tutorials/config/planner_params.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params.yaml) 
```yaml
gripper_name: 'sgb30'
object_mesh_path: '/ros2_ws/src/wrs/pyhiro/suction/objects/ttube.stl'
```
<img src=image/sgb30.gif width=720>  

# Contributors


# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  
