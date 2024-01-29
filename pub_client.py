import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from std_msgs.msg import String

if __name__ == "__main__":
  ws_pub_client = ros_ws.ws_client('pub_client')
  ws_pub_client.advertise('/test_string', String)

  # Test ROS msg publisher
  rate = rospy.Rate(10)

  while not rospy.is_shutdown():
    rospy.spin()
    rate.sleep()