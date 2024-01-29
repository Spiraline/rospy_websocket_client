import websocket
import rel
import json
import yaml
import rospy
import time
from uuid import uuid4

class ws_client():
  def __init__(self, name='', ip='127.0.0.1', port=9090):
    self._name = 'ros_ws_client' if name == '' else name
    rospy.init_node(self._name)
    
    self._ip = ip
    self._port = port
    self._address = "ws://" + self._ip + ":" + str(self._port)
    self._is_connected = False

    self._sub_dict = {}
    self._pub_dict = {}

    self.connect()
    
  # Connect to rosbridge websocket
  def connect(self):
    while not self._is_connected:
      try:
        self._ws = websocket.create_connection(
          'ws://' + self._ip + ':' + str(self._port)
        )
        print("%s:%s\t|\tConnected to server" % (self._ip, self._port))
        self._is_connected = True
      except:
        print("%s:%s\t|\tError connecting to server. Waiting for 5 seconds.." % (ip, port))
        self._connect_flag = False
        time.sleep(5)

  
  def _callback(self, msg, name):
    # Converting ROS message to a dictionary thru YAML
    ros_message_as_dict = yaml.safe_load(msg.__str__())

    json_dict = {
      'name': name,
      'type': self._sub_dict[name]["type"],
      'msg': ros_message_as_dict
    }
    json_msg = json.dumps(json_dict)
    
    try:
      self._ws.send(json_msg)
    except:
      print("%s:%s\t|\Fail to send msg %s" % (self._ip, self._port, name))
      self._is_connected = False
      self.connect()
  
  def advertise(self, topic_name, topic_type):
    type_str = topic_type().__class__.__name__
    self._sub_dict[topic_name] = {'type': type_str,
                                  'subscriber': rospy.Subscriber(topic_name, topic_type, self._callback, topic_name)}

    print("%s:%s\t|\tAdvertise %s with type %s" % (self._ip, self._port, topic_name, type_str))