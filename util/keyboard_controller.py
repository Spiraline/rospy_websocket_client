from pynput import keyboard
import threading
import rospy
from std_msgs.msg import Float32

ACC_MIN = -10
ACC_MAX = 10
STEER_MIN = -750
STEER_MAX = 750

current_pressed = set()
global current_acc
global current_steer
current_acc = 0
current_steer = 0

def on_press(key):
    global current_acc
    global current_steer
    current_pressed.add(key)
    # print('Key %s pressed' % current_pressed)

    if keyboard.Key.up in current_pressed:
        if current_acc < ACC_MAX:
            current_acc += 0.1

    if keyboard.Key.down in current_pressed:
        if current_acc > ACC_MIN:
            current_acc -= 0.1

    if keyboard.Key.left in current_pressed:
        if current_steer < STEER_MAX:
            current_steer += 10

    if keyboard.Key.right in current_pressed:
        if current_steer > STEER_MIN:
            current_steer -= 10

    print('acc : %f steer : %f' %(current_acc, current_steer))

def keyboard_routine():
    with keyboard.Listener(
        on_press=on_press,
        on_release=on_release) as listener:
        listener.join()

def on_release(key):
    if key == keyboard.KeyCode(char='q'):
        print('\nYou pressed q. Quit!')
        return False
    if key in current_pressed:
        current_pressed.remove(key)

if __name__ == '__main__':
    keyboard_thread = threading.Thread(target=keyboard_routine)
    keyboard_thread.start()

    acc_pub = rospy.Publisher('/desired_ax', Float32, queue_size=10)
    steer_pub = rospy.Publisher('/desired_steer', Float32, queue_size=10)

    rospy.init_node('keyboard_test', anonymous=True)
    rate = rospy.Rate(10)

    acc_msg = Float32()
    steer_msg = Float32()
    
    while not rospy.is_shutdown():
        acc_msg.data = current_acc
        steer_msg.data = current_steer

        acc_pub.publish(acc_msg)
        steer_pub.publish(steer_msg)

        rate.sleep()