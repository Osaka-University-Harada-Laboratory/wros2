FROM ubuntu:22.04

# Timezone and Launguage
RUN apt update \
  && apt install -y --no-install-recommends \
     locales \
     software-properties-common tzdata \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && add-apt-repository universe

ENV LANG en_US.UTF-8
ENV TZ=Asia/Tokyo

# Install ROS2
RUN apt update && apt install -y --no-install-recommends \
     git \
     curl \
     byobu \
     gnupg2 \
     lsb-release \
     python3-pip \
     python3-tk \
     vim \
     wget \
     build-essential \
     ca-certificates

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt upgrade -y \
  && DEBIAN_FRONTEND=noninteractive \
  && apt update \
  && apt install -y --no-install-recommends \
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
RUN pip3 install --upgrade setuptools pip
RUN pip3 install .
RUN rm -rf /ros2_ws/src/wrs/setup.py

# Generate bash commands
WORKDIR /ros2_ws/
RUN echo "colcon build --symlink-install --parallel-workers 1" >> ~/.bash_history
RUN echo "ros2 service call /plan_grasp std_srvs/srv/Empty" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py gripper_name:='sgb30' object_mesh_path:='/ros2_ws/src/wrs/pyhiro/suction/objects/ttube.stl'" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py gripper_name:='suction' object_mesh_path:='/ros2_ws/src/wrs/pyhiro/suction/objects/sandpart2.stl'" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py gripper_name:='robotiq140' object_mesh_path:='/ros2_ws/src/wrs/0000_examples/objects/milkcarton.stl'" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py gripper_name:='robotiq85' object_mesh_path:='/ros2_ws/src/wrs/0000_examples/objects/bunnysim.stl'" >> ~/.bash_history
RUN echo "ros2 launch wros2_tutorials plan_grasp_launch.py gripper_name:='robotiqhe' object_mesh_path:='/ros2_ws/src/wrs/0000_examples/objects/tubebig.stl'" >> ~/.bash_history
