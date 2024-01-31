#FROM osrf/ros:noetic-desktop-full
FROM osrf/ros:melodic-desktop-full

#sudo apt-get install python3.7-dev

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-$ROS_DISTRO-rospy-message-converter \
    ros-$ROS_DISTRO-rosbridge-server \
    python3.7-dev \
    python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN python3.7 -m pip install --no-cache-dir --upgrade pip && \
    python3.7 -m pip install --no-cache-dir websocket-client websockets rospkg

CMD ["bin/bash"]
