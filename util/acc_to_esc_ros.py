import rospy
from std_msgs.msg import Float32

class ACC2ESC():
  def __init__(self):
    rospy.init_node('acc_to_esc')
    self.curr_ax = 0.0
    self.discrete_time_integrater = 0.0

    self.accel_msg = Float32()
    self.brake_msg = Float32()

    self.ax_sub = rospy.Subscriber('/desired_ax', Float32, self.axCallback)
    self.accel_pub = rospy.Publisher('/hellocm/accel', Float32, queue_size=1)
    self.brake_pub = rospy.Publisher('/hellocm/brake', Float32, queue_size=1)

    rospy.spin()

  def axCallback(self, msg):
    desired_ax = msg.data

    # Use when stop_req_u1 = true
    # cliped_desried_ax = min(-0.5, desired_ax)

    ax_diff = desired_ax - self.curr_ax

    self.accel_msg.data = 0.05 * ax_diff + self.discrete_time_integrater
    
    # Saturation
    if self.accel_msg.data > 1.0: self.accel_msg.data = 1.0
    elif self.accel_msg.data < -1.0: self.accel_msg.data = -1.0

    if self.accel_msg.data >= 0.0:
      self.brake_msg.data = 0.0
    else:
      self.brake_msg.data = -self.accel_msg.data
      self.accel_msg.data = 0.0
    
    self.discrete_time_integrater += 0.001 * ax_diff * 0.001
    if self.discrete_time_integrater >= 1.0: self.discrete_time_integrater = 1.0
    elif self.discrete_time_integrater < -1.0: self.discrete_time_integrater = -1.0

    self.accel_pub.publish(self.accel_msg)
    self.brake_pub.publish(self.brake_msg)

if __name__ == "__main__":
  acc2esc = ACC2ESC()