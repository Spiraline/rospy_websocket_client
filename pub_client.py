import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from std_msgs.msg import String

if __name__ == "__main__":
  ws_pub_client = ros_ws.ws_client('pub_client')

  # Test ROS msg publisher
  rate = rospy.Rate(10)

  msg = String()
  cnt = 0

  while not rospy.is_shutdown():
    msg.data = "Hi " + str(cnt)
    ws_pub_client.publish('/test_string', msg)
    cnt += 1
    rate.sleep()