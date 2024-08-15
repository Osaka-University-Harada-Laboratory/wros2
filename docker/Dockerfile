FROM ubuntu:22.04

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive

# Timezone and Launguage
RUN apt update -y && apt install -y --no-install-recommends \
    software-properties-common \        
    locales \
    tzdata \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && add-apt-repository universe

ENV LANG en_US.UTF-8
ENV TZ=Asia/Tokyo

RUN apt update -y && apt install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    lsb-release \
    python3-pip \
    python3-tk \
    gnupg2 \
    byobu \
    curl \
    wget \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt upgrade -y && apt update -y && apt install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-tf2-tools \
    ros-humble-tf-transformations \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc 
RUN echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
RUN echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
RUN echo "export XDG_RUNTIME_DIR=/tmp/runtime-futa" >> ~/.bashrc

COPY wrs/ /ros2_ws/src/wrs/
RUN rm -rf /ros2_ws/src/wrs/setup.py /ros2_ws/src/wrs/MANIFEST.in /ros2_ws/src/wrs/requirements.txt
COPY setup.py /ros2_ws/src/wrs/
COPY MANIFEST.in /ros2_ws/src/wrs/
COPY requirements.txt /ros2_ws/src/wrs/
WORKDIR /ros2_ws/src/wrs/
RUN pip3 install setuptools==65.5.0
RUN pip3 install --ignore-installed .
RUN rm -rf /ros2_ws/src/wrs/setup.py

# Generate bash commands
WORKDIR /ros2_ws/
RUN echo "/usr/bin/python3 /ros2_ws/src/wrs/robot_sim/end_effectors/single_contact/suction/sgb30/sgb30.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /ros2_ws/src/wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/sdmbs.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /ros2_ws/src/wrs/robot_sim/end_effectors/gripper/robotiq140/robotiq140.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /ros2_ws/src/wrs/robot_sim/end_effectors/gripper/robotiq85/robotiq85.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /ros2_ws/src/wrs/robot_sim/end_effectors/gripper/robotiqhe/robotiqhe.py" >> ~/.bash_history
RUN echo "colcon build --symlink-install --executor sequential" >> ~/.bash_history
RUN echo "byobu" >> ~/.bash_history
RUN echo "ros2 service call /plan_grasp std_srvs/srv/Empty" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_sgb30_example.yaml" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_suction_example.yaml" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiq140_example.yaml" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiq85_example.yaml" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py config:=planner_params_robotiqhe_example.yaml" >> ~/.bash_history
