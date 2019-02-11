FROM ros:kinetic-ros-core

RUN apt-get -qq update \
    && apt-get install -y libpcl-dev

RUN apt-get -qq update \
    && apt-get install -y libtf2-geometry-msgs-dev

COPY . /home/catkin_ws/src/

RUN rosdep install --as-root apt:yes -r --from-paths /home/catkin_ws/ --ignore-src --rosdistro kinetic -y



SHELL ["bin/bash", "-c"]

RUN source /opt/ros/kinetic/setup.bash \
        && cd /home/catkin_ws/ \
        && catkin_make

CMD cd /home/catkin_ws \
        && source devel/setup.bash \
        && roslaunch ouster_ros os1.launch 