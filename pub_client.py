import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from std_msgs.msg import String

if __name__ == "__main__":
  pub_client = ros_ws.ws_client('pub_client')