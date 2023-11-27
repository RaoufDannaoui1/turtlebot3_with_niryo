#!/usr/bin/env python3
from pyniryo import *
import sys
import rospy
from std_msgs.msg import UInt8


class NiryoConnection:
   def __init__(self):
      rospy.init_node('niryo_connection', anonymous=True)

      self.sub_niryo = rospy.Subscriber('/niryo_con', UInt8, self.niryoCallBack, queue_size = 1)
      self.pub_niryo = rospy.Publisher('/niryo_con', UInt8, queue_size=10)
      self.counter = 0
   def niryoCallBack(self, msg):
      self.counter += 1
      if self.counter<=1:
         print(msg.data)
         if msg.data == 1:
            robot = NiryoRobot("192.168.0.150")

            # robot.calibrate_auto()

            try:
               # Move to an observation position then
               robot.move_pose(*[0.001, -0.213, 0.217, 3.1, 1.395, 1.559])
               # Try to do a vision pick:
               if robot.vision_pick('default_workspace_turltelbot', 0/1000.0, ObjectShape.CIRCLE, ObjectColor.RED)[0]:
                  # If an object has been taken, do:
                  robot.place_from_pose(*[0.326, -0.015, 0.314, -2.232, 1.471, -2.234])
                  robot.move_pose(*[0.326, -0.015, 0.364, -2.175, 1.476, -2.178])
                  robot.move_pose(*[0, -0.284, 0.325, 2.928, 1.346, 1.383])

            except NiryoRosWrapperException as e:
               sys.stderr.write(str(e))

            robot.close_connection()
            self.pub_niryo.publish(2)
            # rospy.signal_shutdown("Task completed")  # Shutdown the ROS node after completing the task


if __name__ == '__main__':
   try:
      niryoCon = NiryoConnection()
      rospy.spin()

   except rospy.ROSInterruptException:
      pass

