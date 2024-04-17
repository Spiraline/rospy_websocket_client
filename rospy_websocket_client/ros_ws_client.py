import websockets
import asyncio
import json
import yaml
import rospy
import time
from rospy_message_converter import message_converter

class ws_client():
  def __init__(self, name='', ip='127.0.0.1', port=9090):
    self._name = 'ros_ws_client' if name == '' else name
    rospy.init_node(self._name)
    
    self._ip = ip
    self._port = port
    self._address = "ws://" + self._ip + ":" + str(self._port)
    self._ws = None

    self._advertise_dict = {}
    self._sub_dict = {}
    
  # Connect to rosbridge websocket
  async def connect(self):
    fail_cnt = 0
    while not self._ws:
      if fail_cnt > 5:
        print("%s:%s\t|\tError connecting to server for 10s. Exit" % (self._ip, self._port))
        exit(1)
      try:
        self._ws = await websockets.connect(
          uri = self._address,
          timeout = 1
        )
        print("%s:%s\t|\tConnected to server" % (self._ip, self._port))
      except:
        print("%s:%s\t|\tError connecting to server. Waiting for 2 seconds.. (%d / 5)" % (self._ip, self._port, fail_cnt))
        fail_cnt += 1
        time.sleep(2)
  
  async def close(self):
    if self._ws is not None:
      await self._ws.close()

  async def recv(self):
    try:
      json_raw = await asyncio.wait_for(self._ws.recv(), timeout=0.01)
    except:
      print("%s:%s\t|\tFail to recv msg" % (self._ip, self._port))
      return None
    
    json_msg = json.loads(json_raw)

    if json_msg['op'] == 'publish':
      msg_dict = json_msg['msg']
      topic_name = json_msg['topic']
      topic_type = self._sub_dict[topic_name]['msg']._type
      ros_msg = message_converter.convert_dictionary_to_ros_message(topic_type, msg_dict)

      return ros_msg

    return None
  
  async def _advertise(self, topic_name, topic_type):
    """
    Advertise a topic with it's type in 'package/Message' format.
    :param str topic_name: ROS topic name.
    :param str topic_type: ROS topic type, e.g. std_msgs/String.
    """
    self._advertise_dict[topic_name] = {'topic_type': topic_type}
    advertise_msg = {"op": "advertise",
                      "topic": topic_name,
                      "type": topic_type
                      }

    try:
      await self._ws.send(json.dumps(advertise_msg))
    except:
      print("%s:%s\t|\Fail to advertise msg %s" % (self._ip, self._port, topic_name))
      if self._ws.closed:
        self.connect()

    print("%s:%s\t|\tAdvertise %s with type %s" % (self._ip, self._port, topic_name, topic_type))
    
  async def _subscribe(self, topic_name, msgs_data, rate = 0.0, queue_size = 0):
    _rate = 0.0

    if rate > 0.0:
        _rate = 1000.0 / rate

    pub_msg = {
      'op': 'subscribe',
      'topic': topic_name,
      'msgs_data': msgs_data._type,
      'throttle_rate': int(_rate),
      'queue_length': int(queue_size)
    }

    # send to server
    json_msg = json.dumps(pub_msg)
    await self._ws.send(json_msg)
    print("%s:%s\t|\tSubscribe to: %s \ttype: %s \trate: %s \tqueue_length: %s" % (self._ip, self._port, topic_name, msgs_data._type, rate, queue_size))
  
  async def subscribe(self, topic_name, msgs_data, rate = 0, queue_size = 0):
    # If not advertised, advertise topic
    if topic_name not in self._advertise_dict:
      topic_type = msgs_data._type
      await self._advertise(topic_name, topic_type)

    pub = rospy.Publisher(topic_name, msgs_data.__class__, queue_size=queue_size)
    self._sub_dict[topic_name] = {
      'msg': msgs_data,
      'publisher': pub,
      'rate': rate,
      'queue_size': queue_size
    }

    await self._subscribe(topic_name, msgs_data, rate, queue_size)

  async def publish(self, topic_name, ros_message):
    # If not advertised, advertise topic
    if topic_name not in self._advertise_dict:
      topic_type = ros_message._type
      await self._advertise(topic_name, topic_type)

    ros_message_as_dict = yaml.safe_load(ros_message.__str__())
    
    msg = {
      'op': 'publish',
      'topic': topic_name,
      'msg': ros_message_as_dict
    }
    json_msg = json.dumps(msg)

    try:
      await self._ws.send(json_msg)
      print("%s:%s\t|\tPublish msg %s" % (self._ip, self._port, topic_name))
    except:
      print("%s:%s\t|\tFail to publish msg %s" % (self._ip, self._port, topic_name))
      if self._ws.closed:
        self.connect()