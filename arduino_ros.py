# #!/usr/bin/env python3
# import rospy
# from std_msgs.msg import Int16MultiArray
# import sys
# import select
# import termios
# import tty

# class RobotController:
#     def __init__(self):
#         rospy.init_node('robot_dr      iver_node', anonymous=True)
        
#         self.sonar_data = [0, 0, 0, 0]

#         self.pub_led = rospy.Publisher('ledControl', Int16MultiArray, queue_size=10)
#         self.pub_cyl = rospy.Publisher('cylinderControl', Int16MultiArray, queue_size=10)

#         rospy.Subscriber('sonarSensor', Int16MultiArray, self.sonar_cb)
        
#         self.door1_state = 2
#         self.door2_state = 2

#         self.is_blinking = False       
#         self.blink_state = False       
#         self.last_blink_time = 0       

#         print("Connection Ros-Noetic Successful!")

#     def sonar_cb(self, msg):
#         self.sonar_data = msg.data

#     def send_led(self, led_list):
#         self.is_blinking = False 
#         msg = Int16MultiArray()
#         msg.data = led_list
#         self.pub_led.publish(msg)

#     def start_blinking(self):
#         self.is_blinking = True

#     def update_blink_logic(self):
#         if self.is_blinking:
#             current_time = rospy.get_time()
#             if current_time - self.last_blink_time > 0.5:
#                 self.blink_state = not self.blink_state 
#                 val = 0 if self.blink_state else 1
#                 msg = Int16MultiArray()
#                 msg.data = [val, val, val, val]
#                 self.pub_led.publish(msg)
#                 self.last_blink_time = current_time

#     def send_cylinder(self, pair1, pair2):
#         self.door1_state = pair1
#         self.door2_state = pair2

#         msg = Int16MultiArray()
#         msg.data = [pair1, pair2]
#         self.pub_cyl.publish(msg)

# def getKey():
#     tty.setraw(sys.stdin.fileno())
#     rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
#     if rlist:
#         key = sys.stdin.read(1)
#     else:
#         key = ''
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key

# if __name__ == '__main__':
#     settings = termios.tcgetattr(sys.stdin)
#     robot = RobotController()

#     print("""
#     =============================================
#     Press the key to control.
#     =============================================
#     [a] Open/Close Door 1
#     [d] Open/Close Door 2
#     ---------------------------------------------
#     [o] Open Door 1 & Door 2
#     [c] Close Door 1 & Close 2
#     [s] Emergency stop Door 1 & Close 2
#     ---------------------------------------------
#     [l] On led
#     [k] Off led
#     [j] Blinking led
#     ---------------------------------------------
#     [e] Exit
#     """)

#     rate = rospy.Rate(10) 

#     try:
#         while not rospy.is_shutdown():
#             robot.update_blink_logic()

#             sys.stdout.write(f"\r[Sonar_sensor]: {list(robot.sonar_data)} | Door1: {robot.door1_state} | Door2: {robot.door2_state}   ")
#             sys.stdout.flush()

#             key = getKey()
            
#             if key == 'a':
#                 new_state = 0 if robot.door1_state == 1 else 1
#                 robot.send_cylinder(new_state, robot.door2_state)

#             elif key == 'd':
#                 new_state = 0 if robot.door2_state == 1 else 1
#                 robot.send_cylinder(robot.door1_state, new_state)

#             elif key == 'o':
#                 robot.send_cylinder(1, 1)

#             elif key == 'c':
#                 robot.send_cylinder(0, 0)

#             elif key == 's':
#                 robot.send_cylinder(2, 2)

#             elif key == 'j': 
#                 robot.start_blinking()
            
#             elif key == 'l': 
#                 robot.send_led([0, 0, 0, 0])

#             elif key == 'k': 
#                 robot.send_led([1, 1, 1, 1])
            
#             elif key == 'e': 
#                 break
            
#             rate.sleep()

#     except Exception as e:
#         print(e)

#     finally:
#         robot.send_cylinder(2, 2)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#         print("\nGoodbye!")

#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16MultiArray
import sys

class RobotMonitor:
    def __init__(self):
        rospy.init_node('robot_monitor_node', anonymous=True)
        
        self.sonar_data = [0, 0, 0, 0]
        self.door_states = [2, 2]
        self.led_states = [1, 1, 1, 1]

        # Subscribe to relevant topics
        rospy.Subscriber('sonarSensor', Int16MultiArray, self.sonar_cb)
        rospy.Subscriber('cylinderControl', Int16MultiArray, self.cyl_status_cb)
        rospy.Subscriber('ledControl', Int16MultiArray, self.led_status_cb)

        print("--- MONITOR NODE ACTIVE ---")

    def sonar_cb(self, msg):
        self.sonar_data = msg.data

    def cyl_status_cb(self, msg):
        self.door_states = msg.data

    def led_status_cb(self, msg):
        self.led_states = msg.data

if __name__ == '__main__':
    robot = RobotMonitor()
    rate = rospy.Rate(10) 

    try:
        while not rospy.is_shutdown():
            led_display = ["ON" if x == 0 else "OFF" for x in robot.led_states]
            
            # Print the monitored data
            sys.stdout.write(
                f"\r[MONITOR] Sonar: {list(robot.sonar_data)} | "
                f"Doors: {list(robot.door_states)} | "
                f"LEDs: {led_display}   "
            )
            sys.stdout.flush()
            rate.sleep()
    except Exception as e:
        print(e)