import argparse
import asyncio
import rospy
from rospy_websocket_client import ros_ws_client as ros_ws
from std_msgs.msg import String

async def main(client):
  try:
    await client.connect()
    await client.subscribe('/test_string', String(), 10, 1)

    while not rospy.is_shutdown():
      msg = await client.recv()
      if msg != None:
        print(msg)
  except KeyboardInterrupt:
    print("Received Ctrl+C, closing connection")
  finally:
    await client.close()

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='pub_client')
  parser.add_argument('--ip', '-i', type=str, default='127.0.0.1')
  parser.add_argument('--port', '-p', type=str, default='9090')

  args = parser.parse_args()

  ws_sub_client = ros_ws.ws_client('sub_client', args.ip, args.port)

  try:
    asyncio.run(main(ws_sub_client))
  except rospy.ROSInterruptException:
    pass