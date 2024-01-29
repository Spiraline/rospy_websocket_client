# rospy_websocket_client

Publish/Subscribe topic via rosbridge websocket server

## Requirement
- Python 3.7+
- ROS 1 (tested in melodic and noetic)

- apt packages
  ```
  sudo apt-get install ros-$ROS_DISTRO-rospy-message-converter
  sudo apt-get install ros-$ROS_DISTRO-rosbridge-server
  ```

- python packages
  ```
  python3.7 -m pip install websocket-client rospkg 
  ```

## Usage
**[PC 1]**
```
roslaunch rosbridge_server rosbridge_websocket.launch

python3.7 sub_client.py
```

**[PC 2]**
```
python3.7 pub_client.py
```