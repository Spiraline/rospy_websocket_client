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

    # Connect to rosbridge websocket
    while not self._is_connected:
      try:
        self._ws = websocket.create_connection(
          'ws://' + self._ip + ':' + str(self._port)
        )
        print("%s:%s\t|\tConnected to server" % (ip, port))
        self._is_connected = True
      except:
        print("%s:%s\t|\tError connecting to server. Waiting for 5 seconds.." % (ip, port))
        self._connect_flag = False
        time.sleep(5)
  
  