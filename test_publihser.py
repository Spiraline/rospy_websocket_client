import rospy
from std_msgs.msg import String

if __name__ == "__main__":
  rospy.init_node("test_publihser")

  # Test ROS msg publisher
  test_pub = rospy.Publisher('/test_string', String, queue_size=1)
  rate = rospy.Rate(10)
  msg = String()
  msg.data = "test"

  while not rospy.is_shutdown():
    test_pub.publish(msg)
    rate.sleep()