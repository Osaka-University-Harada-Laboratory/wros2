# wros2

ROS2 node examples with WRS.

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/wros2)

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
git clone git@github.com:Osaka-University-Harada-Laboratory/wros2.git --recursive --depth 1 && cd wros2
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

# Usage
1. Build and run the docker environment
- Create and start docker containers in the initially opened terminal
  ```bash
  docker compose up
  ```
- Execute the container in another terminal
  ```bash
  xhost +
  docker exec -it wros_humble_container bash
  ```
2. Change planning parameters in wros2_tutorials/config/XXX.yaml 
3. Build program files with the revised yaml
    ```bash
    colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
    ```
4. Run a planning process in the container  
    ```bash
    byobu
    ros2 launch wros2_tutorials plan_grasp_launch.py config:=XXX.yaml
    # F2 to create a new window
    ros2 service call /plan_grasp std_srvs/srv/Empty
    # Ctrl + F6 to close the selected window
    ```

## [Robotiq Hand-E](https://robotiq.com/products/hand-e-adaptive-robot-gripper)
Please refer to [wros2_tutorials/config/planner_params_robotiqhe_example.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params_robotiqhe_example.yaml).
```bash
# byobu
ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiqhe_example.yaml
# F2
# ros2 service call /plan_grasp std_srvs/srv/Empty
```  
<img src=image/robotiqhe.gif width=720>  

## [Robotiq 2F-85](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
Please refer to [wros2_tutorials/config/planner_params_robotiq85_example.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params_robotiq85_example.yaml).
```bash
# byobu
ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiq85_example.yaml
# F2
# ros2 service call /plan_grasp std_srvs/srv/Empty
```  
<img src=image/robotiq85.gif width=720>  

## [Robotiq 2F-140](https://robotiq.com/products/2f85-140-adaptive-robot-gripper)
Please refer to [wros2_tutorials/config/planner_params_robotiq140_example.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params_robotiq140_example.yaml).
```bash
# byobu
ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiq140_example.yaml
# F2
# ros2 service call /plan_grasp std_srvs/srv/Empty
```  
<img src=image/robotiq140.gif width=720>  

## Suction gripper
Please refer to [wros2_tutorials/config/planner_params_suction_example.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params_suction_example.yaml).
```bash
# byobu
ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_suction_example.yaml
# F2
# ros2 service call /plan_grasp std_srvs/srv/Empty
```  
<img src=image/suction.gif width=720>  

## [CONVUM balloon hand SGB30](https://convum.co.jp/products/en/other-en/sgb/)
Please refer to [wros2_tutorials/config/planner_params_sgb30_example.yaml](ros2_ws/humble/src/wros2_tutorials/config/planner_params_sgb30_example.yaml).
```bash
# byobu
ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_sgb30_example.yaml
# F2
# ros2 service call /plan_grasp std_srvs/srv/Empty
```  
<img src=image/sgb30.gif width=720>  

# Contributors

We always welcome collaborators!

# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
