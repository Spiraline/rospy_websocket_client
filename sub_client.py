import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from std_msgs.msg import String

if __name__ == "__main__":
  ws_sub_client = ros_ws.ws_client('sub_client')
  ws_sub_client.subscribe('/test_string', String(), 10, 1)

  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    msg = ws_sub_client.recv()
    print(msg.data)
    rate.sleep()