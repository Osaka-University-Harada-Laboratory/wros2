# wros2

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/wros2)

ROS2 node examples with WRS.

- ROS Humble Hawksbill node examples with robot motion planners implemented in [WRS](https://github.com/wanweiwei07/wrs).

# Features

- Docker environment for ROS Humble Hawksbill packages
- ROS2 node examples with [grasp planners (Wan et al., IEEE TRO 2021)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9170578)

# Dependency (tested as a host machine)

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - NVIDIA GeForce RTX 3070
  - NVIDIA Driver 470.256.02
  - Docker 26.1.1
  - Docker Compose 2.27.0
  - NVIDIA Docker 2.13.0

# Installation
```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/wros2.git --recursive --depth 1 && cd wros2 && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

# Usage
1. Build and run the docker environment
   - Create and start docker containers in the initially opened terminal
        ```bash
        docker compose up
        ```
   - Execute the container in another terminal
        ```bash
        xhost + && docker exec -it wros2_humble_container bash
        ```
2. Change planning parameters in wros2_tutorials/config/XXX.yaml 
3. Build program files with the revised yaml
    ```bash
    cd /ros2_ws && colcon build --symlink-install --parallel-workers 1 && source install/setup.bash
    ```
4. Run a planning process in the container
   - Use byobu to easily command several commands  
        ```bash
        byobu
        ```
        - First command & F2 to create a new window & Second command ...
        - Ctrl + F6 to close the selected window
   - Run the grasp planner  
        ```bash
        ros2 launch wros2_tutorials plan_grasp_launch.py config:=XXX.yaml
        ```
   - Call the planning service  
        ```bash
        ros2 service call /plan_grasp std_srvs/srv/Empty
        ```

#### Please refer to [wiki page](https://github.com/Osaka-University-Harada-Laboratory/wros2/wiki/Usage-examples) for usage examples.

# Contributors

We always welcome collaborators!

# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  

## License

This software is released under the BSD-3-Clause License, see [LICENSE](./LICENSE).
