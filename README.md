# rospy_websocket_client

Publish/Subscribe topic via rosbridge websocket server

## Requirement
- Python 3.7+
- ROS 1 (tested in melodic and noetic)
  - Ubuntu 18.04 should install python3.7 since it has python3.6 in default
  - Ubuntu 20.04 has python3.7+, so you can execute with `python3` instead of `python3.7`


- apt packages
  ```
  sudo apt-get install ros-$ROS_DISTRO-rospy-message-converter
  sudo apt-get install ros-$ROS_DISTRO-rosbridge-server
  ```

- python packages
  ```
  python3.7 -m pip install websocket-client
  ```

- Firewall setting (for PC runs `rosbridge_websocket`)
  ```
  sudo ufw allow 9090
  ```

- ping to check if two PCs are connected

## Example
**[PC 1] (ROS melodic)**
```
roslaunch rosbridge_server rosbridge_websocket.launch --address:="$(PC 1's ip address)"

python3.7 pub_client.py -i $(PC 1's ip address)
```

**[PC 2] (ROS noetic)**
```
roscore

python3 sub_client.py -i $(PC 1's ip address)
```