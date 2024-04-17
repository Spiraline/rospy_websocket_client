import argparse
import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from util.acc_to_esc import ACC2ESC
from std_msgs.msg import Float32
from geometry_msgs.msg import AccelStamped, TwistStamped, PoseStamped

GPS_POSE_TOPIC = '/carmaker/gps_pose'
GPS_TWIST_TOPIC = '/carmaker/gps_twist'
GPS_ACCEL_TOPIC = '/carmaker/gps_accel'

ESC_ACCEL_TOPIC = '/carmaker/accel'
ESC_BRAKE_TOPIC = '/carmaker/brake'
EPS_STEER_TOPIC = '/carmaker/steer'

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='rf_client')
  parser.add_argument('--ip', '-i', type=str, default='127.0.0.1')
  parser.add_argument('--port', '-p', type=str, default='9090')

  args = parser.parse_args()

  acc2esc = ACC2ESC()

  rf_client = ros_ws.ws_client('rf_client', args.ip, args.port)

  rf_client.subscribe(GPS_POSE_TOPIC, PoseStamped(), 10, 1)
  rf_client.subscribe(GPS_TWIST_TOPIC, TwistStamped(), 10, 1)
  rf_client.subscribe(GPS_ACCEL_TOPIC, AccelStamped(), 10, 1)

  rate = rospy.Rate(100)

  while not rospy.is_shutdown():
    topic_name, msg = rf_client.recv()
    if msg != None:
      if topic_name == GPS_ACCEL_TOPIC:
        ax = msg.accel.linear.x
        acc2esc.setCurrAx(ax)

        try:
          ax_msg = rospy.wait_for_message('/desired_ax', Float32, timeout=0.1)
          
          acc2esc.axCallback(ax_msg.data)
          accel_msg = Float32()
          accel_msg.data = acc2esc.getAccel()
          brake_msg = Float32()
          brake_msg.data = acc2esc.getBrake()

          rf_client.publish(ESC_ACCEL_TOPIC, accel_msg)
          rf_client.publish(ESC_BRAKE_TOPIC, brake_msg)

          print(acc2esc.getcurrAx(), ax_msg.data)
          print(accel_msg.data, brake_msg.data)
        except:
          # print('No desired_ax msg received')
          pass
        
        try:
          desired_steer = rospy.wait_for_message('/desired_steer', Float32, timeout=0.1)
          steer_msg = Float32()
          steer_msg.data = desired_steer.data
          rf_client.publish(EPS_STEER_TOPIC, steer_msg)
        except:
          # print('No desired_steer msg received')
          pass
        
    rate.sleep()