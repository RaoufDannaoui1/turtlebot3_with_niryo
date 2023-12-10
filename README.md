# Autonomous Robotic Coordination with TurtleBot3 and Niryo Ned2
<div style="text-align: center">
  <img src="images/main.png"/>
</div>

Authors:
  - Abdel Raouf Dannaoui 
  - Nouriddin Asfour

Supervisors:
 - Joaquin Jorge Rodriguez
 - Raphael Duverne


## Table of Contents
1. [Project Goal](#project-goal)
2. [Project Workflow](#project-workflow)
3. [Required Libraries and Packages](#required-libraries-and-packages)
4. [Installing the Required Packages](#installing-the-required-packages)
5. [TurtleBot3 Configuration](#turtlebot3-configuration)
    - [Connection](#tb3-connection)
    - [Intrinsic Camera Calibration](#intrinsic-camera-calibration)
    - [Extrinsic Camera Calibration](#extrinsic-camera-calibration)
    - [Check Calibration Result](#check-calibration-result)
    - [Lane Detection](#lane-detection)
    - [ArUco Tag Detection](#aruco-tag-detection)
    - [Drive Controller](#drive-controller)
6. [Niryo Ned 2 Configuration](#niryo-ned-2-configuration)
    - [Connection](#niryo-connection)
    - [Calibration](#calibration)
    - [Workspace Creation](#workspace-creation)
    - [Program](#program)
7. [Autonomous Robotic Coordination](#autonomous-robotic-coordination)
    - [Expected Result](#expected-result)
    - [Running the Program](#running-the-program)
8. [Demo Videos](#demo-videos)



## Project Goal
<b>Experience seamless robotic collaboration with this project!</b><br>

Our TurtleBot3 autonomously navigates between yellow and white lines based on [Autorace Challenge][1], pausing when detecting an ArUco tag at a specific distance. Once halted, it signals Niryo to execute a precise pick-and-place task. After completion, TurtleBot3 resumes its journey until it encounters another ArUco tag, marking the end of the simulation. Dive into the world of robotic coordination for efficient and intelligent task execution.




## Project Workflow 
<div style="text-align: center">
  <img src="images/flowchart.png"/>
</div>




## Required Libraries and Packages
- ros-noetic-image-transport 
- ros-noetic-cv-bridge 
- ros-noetic-vision-opencv 
- python3-opencv 
- libopencv-dev 
- ros-noetic-image-proc
- pyniryo
- pyniryo2




## Installing the Required Packages
For this step, we followed the instructions provided on [emanual robotics][1], then manipulated those packages to match our project goal. Installing our edited AutoRace 2020 meta package on your Remote PC lets you start.

```bash
cd ~/catkin_ws/src/
git clone https://github.com/RaoufDannaoui1/turtlebot3_with_niryo.git
cd ~/catkin_ws && catkin_make
```     
Install additional dependent packages on the Remote PC.

```bash
sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc
pip install pyniryo
pip install pyniryo2
```



## TurtleBot3 Configuration
<div style="text-align: center">
  <img src="images/turtlebot3_burger.png"/>
</div>

### TB3 Connection
Connect to the turtlebot3 through SSH on a reserved specific IP on the router and use `napelturbot` as the password.
```bash
ssh ubuntu@{IP_ADDRESS_OF_RASPBERRY_PI}
```
open `.bashrc` file and check for the below lines

```bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_HOSTNAME={IP_ADDRESS_OF_RASPBERRY_PI}
```

Save and source it using 

```bash
source ./bashrc
```

Once connected, do not forget to bring up your robot.

`TB3`
```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch 
```

### Intrinsic Camera Calibration
Here we used a big Checkerboard (size 8x6) to perform the intrinsic camera calibration. First, we should turn on the camera on the TB3.

`Remote PC`
```bash
roscore
```
`TB3`
```bash
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```     

After launching the camera node, you can start the calibration.

`Remote PC`
```bash
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/image camera:=/camera
```

Move the board to the x-axis, y-axis and, z-axis. Continue moving the board till the 4 bars get fully green. When successfully getting the bars green press on calibrate button as shown below.

<div style="text-align: center">
  <img src="images/int_calib_calib.png"/>
</div>

Now click on the Save button as shown below  to save the calibration data. Then a `calibrationdata.tar.gz` folder will be created in `/tmp` folder. Extract `calibrationdata.tar.gz` folder, and open `ost.yaml`.

<div style="text-align: center">
  <img src="images/int_calib_save.png"/>
</div>

 Now copy and paste the data from `ost.yaml` to `camerav2_320x240_30fps.yaml` under the `intrinstic_calibration` folder in the `turtlebo3_autorace_camera` package.

<div style="text-align: center">
  <img src="images/int_calib_data.png"/>
</div>




### Extrinsic Camera Calibration
The extrinsic camera calibration is quite straightforward. To detect the lanes as far as possible, we should adjust the projection view of the camera by adjusting the coordinates of the projection rectangle.


`Remote PC`
```bash
roscore
```

`TB3`
```bash
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```     

After launching the camera node, you can start the calibration. Launch the intrinsic camera calibration node in action mode. 

`Remote PC`
```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
```

Then Launch the extrinsic camera calibration node in calibration mode.

`Remote PC`
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=calibration
```

Now to visualise the calibration use 

`Remote PC`
```bash
    rqt_image_view
```

Open multiple monitors and select `/camera/image_extrinsic_calib/compressed` and `/camera/image_projected_compensated` topics on each monitor. One of the two screens will show an image with a red rectangle box. The other one shows the ground projected view (Bird’s eye view).

<div style="text-align: center">
  <img src="images/ext_calib_calib.png"/>
</div>

Excute rqt reconfiguration 

`Remote PC`
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

Adjust parameters in `/camera/image_projection` and `/camera/image_compensation_projection`

<div style="text-align: center">
  <img src="images/ext_calib_data.png"/>
</div>

When you are satisfied with the results save the new values in `image_projection.yaml` and `compensation.yaml` under the `extrinstic_calibration` folder in the `turtlebo3_autorace_camera` package.




### Check Calibration Result
After completing calibrations, run the step-by-step instructions below on `Remote PC` to check the calibration result.


Run the intrinsic camera calibration launch file
```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
```

Run the extrinsic camera calibration launch file
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
```

Run the rqt to visualize the results
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

<div style="text-align: center">
  <img src="images/int_ext_res.png"/>
</div>




### Lane Detection
The lane detection package that runs on the `Remote PC` receives camera images from TurtleBot3 to detect driving lanes and to drive the TurtleBot3 along them.
The following instructions describe how to use and calibrate the lane detection feature via rqt

Launch roscore.

`Remote PC`
```bash
  roscore
```

Trigger the camera.

`TB3`
```bash
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```

Run an intrinsic camera calibration launch file on the Remote PC.

`Remote PC`
```bash
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action
```

Run an extrinsic camera calibration launch file on the Remote PC.

`Remote PC`
```bash
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action
```

Run a lane detection launch file on the Remote PC

`Remote PC`
```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=calibration
```

Now to visualise the calibration use 

`Remote PC`
```bash
rqt_image_view
```

Select three topics at each image view: `/detect/image_yellow_lane_marker/compressed`, `/detect/image_lane/compressed`, `/detect/image_white_lane_marker/compressed`

<div style="text-align: center">
  <img src="images/lane_calib.png"/>
</div>

The left (Yellow line) and the center (White line) screen show a filtered image. The right screen is the view of the camera from TurtleBot3.

Excute rqt reconfiguration 

`Remote PC`
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
<div style="text-align: center">
  <img src="images/lane_calib_calib.png"/>
</div>

By default, the Turtlebot3 burger detects lanes by identifying the left yellow lane and the
right white lane on gray or black ground. For the yellow and white colors, we have to tune the HSL parameters for the 2 colors. After several adjustments, the blurry lanes became identified

<div style="text-align: center">
  <img src="images/lane_calib_res.png"/>
</div>

Open `lane.yaml` file located in `turtlebot3_autorace_detect/param/lane/`. You need to write modified values to the file. This will make the camera set its parameters as you set here from next launching

<div style="text-align: center">
  <img src="images/lane_calib_data.png"/>
</div>

Check the results by running 

`Remote PC`
```bash
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action
```

Run rqt
`Remote PC`
```bash
rqt_image_view
```

### Aruco tag detection

Using the camera on the TB3 we created an aruco tag detector which will detect the tag and publish on `/aruco_distance` topic the distance between the robot and the aruco tag.



| **Aruco Marker** | **Detected Marker** |
| -------------------- | -------------------- |
| ![Aruco Marker](images/aruco_tag.jpg) | ![Detected Marker](images/aruco_tag_detected.jpg) |

```python
#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge
from std_msgs.msg import Float64
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv2 import aruco

class ArucoDetection:
    def __init__(self):
        rospy.init_node('aruco_detection', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)
        self.aruco_pub = rospy.Publisher('/aruco_markers', Image, queue_size=10)
        self.aruco_distance_pub = rospy.Publisher('/aruco_distance', Float64, queue_size=10)

        self.camera_matrix = np.array([[153.53337,   0.     , 150.50761],
                                      [0.       , 153.62598, 118.64754],
                                      [0.       ,   0.     ,   1.     ]])
        
        self.distortion_coefficients = np.array([-0.318094, 0.090092, 0.000346, -0.000410, 0.0])

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()
        
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    
            if ids is not None:
                for i in range(len(ids)):
                    # print(f"Detected ArUco marker {ids[i]}")
                    rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.distortion_coefficients)

                    aruco.drawDetectedMarkers(cv_image, corners, ids)

                    distance_to_tag = np.linalg.norm(translation_vectors[0])
                    print(f"detected marker {ids[i]} at distance {distance_to_tag} meters")

                    self.aruco_distance_pub.publish(Float64(distance_to_tag))
                
                # Convert the image back to ROS format and publish it
                aruco_image_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
                self.aruco_pub.publish(aruco_image_msg)

        except Exception as e:
            rospy.logerr(f"Error processing image: {str(e)}")

if __name__ == '__main__':
    try:
        aruco_detection = ArucoDetection()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass      
```
In this code, we used `open-cv` to have the `aruco` features. Then from the intrinsic camera calibration, we used the `camera matrix` and the `distortion coefficients` to have the exact distance between the tag and the TB3. Now this distance will be published on `/aruco_distance topic`


### Drive Controller

In this controller we are subscribed to the `/aruco_distance topic`, after the autonomous drive of the TB3 and arriving at the aruco area, it will check if the distance is satisfied.
Then it will decided based on the need of the Niryo job, if needed it will stop for the Niryo to send a signal over `/niryo_con` topic to do his task. When Niryo Finishes his task, he will resend another signal on the same `/niryo_con` topic telling the TB3 to continue the autonomous drive. If Niryo Job is not needed The TB3 will stop.

```python
#!/usr/bin/env python 
import rospy
import numpy as np
from std_msgs.msg import UInt8 ,Float64
from geometry_msgs.msg import Twist

from pyniryo import *
import sys


class ControlLane():
    def __init__(self):
        self.sub_lane = rospy.Subscriber('/control/lane', Float64, self.cbFollowLane, queue_size = 1)
        self.sub_max_vel = rospy.Subscriber('/control/max_vel', Float64, self.cbGetMaxVel, queue_size = 1)
        self.pub_cmd_vel = rospy.Publisher('/control/cmd_vel', Twist, queue_size = 1)

        self.sub_aruco_distance = rospy.Subscriber('/aruco_distance', Float64, self.arucoCallBack, queue_size = 1)
        
        self.sub_niryo = rospy.Subscriber('/niryo_con', UInt8, self.niryoCallBack, queue_size = 1)
        self.pub_niryo = rospy.Publisher('/niryo_con', UInt8, queue_size=10)


        self.lastError = 0
        self.MAX_VEL = 0.1

        self.stopped = False
        self.niryoJobWanted = True

        rospy.on_shutdown(self.fnShutDown)

    def cbGetMaxVel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def arucoCallBack(self, msg):
        distance_to_tag = msg.data
        if 0.42 < distance_to_tag < 0.45:
            self.stopped = True
    
    def niryoCallBack(self, msg):
        if msg.data == 2:
            self.stopped = False
            self.niryoJobWanted = False

    def cbFollowLane(self, desired_center):
        if not self.stopped:
            print("driving")
            center = desired_center.data

            error = center - 500

            Kp = 0.0025
            Kd = 0.007

            angular_z = Kp * error + Kd * (error - self.lastError)
            self.lastError = error
            
            twist = Twist()
            # twist.linear.x = 0.05        
            twist.linear.x = min(self.MAX_VEL * ((1 - abs(error) / 500) ** 2.2), 0.05)
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.pub_cmd_vel.publish(twist)

        elif self.niryoJobWanted:
            print("stopped for niryo")
            self.pub_niryo.publish(1)

        else:
            print("finished job")
            self.stopped = True


    def fnShutDown(self):
        rospy.loginfo("Shutting down. cmd_vel will be 0")

        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        self.pub_cmd_vel.publish(twist) 

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('control_lane')
    node = ControlLane()
    node.main()
```


## Niryo Ned 2 Configuration
<div style="text-align: center">
  <img src="images/niryo_ned2.webp"/>
</div>

### Niryo Connection

Before Connecting to the Niro, we should reserve an IP for it on the router, and then connect to it through SSH on the specific IP and use `roboticcs` as a password, in my case, I am using ethernet to connect.
```bash
ssh niryo@{IP_ADDRESS_OF_NIRYO}
```

open `.bashrc` file and add 

```bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_HOSTNAME={IP_ADDRESS_OF_NIRYO}
```

Save and source it using 

```bash
source ./bashrc
```

Now open the Niryo Studion application on your PC and connect to the robot

<div style="text-align: center">
  <img src="images/niryo_studio.png"/>
</div>

1. The top toolbar allows you to change the angle units, and the language, and connect to Ned
2. 3D Ned visualization or camera stream video
3. State section and mode selection
4. The left menu allows you to switch between the different sections of the application
5. The main window of the application

### Calibration

Since Niryo is a smart manipulator it has an auto-calibration system with just one click.

### Workspace Creation

Before utilizing the Vision functions, it's necessary to establish the workspace(s) you intend to employ. You have the option to save multiple workspaces, so if you're working with a pre-existing one, there's no need to go through the following procedure again.

In Niryo Studio’s menu, open the Vision tab and click the button to add a new workspace. You will be guided, step by step, to set a workspace. Be sure to point the markers correctly in the right order!

<div style="text-align: center">
  <img src="images/niryo_workspace.png"/>
</div>

### Program

Click the project button on the left menu to start creating and editing Blocky programs.

<div style="text-align: center">
  <img src="images/niryo_program.png"/>
</div>

From here we select The `vision` tab and select a template to pick-and-place a block

So after getting the template we can save it and get the `python` code behind it

```python
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
      self.jobDone = False
   def niryoCallBack(self, msg):
      self.jobDone = True
      if not self.jobDone:
         print(msg.data)
         if msg.data == 1:
            robot = NiryoRobot("192.168.0.150")

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

if __name__ == '__main__':
   try:
      niryoCon = NiryoConnection()
      rospy.spin()

   except rospy.ROSInterruptException:
      pass
```
After some manipulation, the `Niryo` will be subscribing to `/niryo_con` common topic between both robots. When the TB3 detects the `aruco tag` it will perform the pick-and-place task and then through the publisher on the same topic, the TB3 will know that he finished and continue driving to its end position.

## Autonomous Robotic Coordination
### Expected Result
After setting both robots, now the `TB3` can drive from the starting point to a specific area where there's an aruco tag at a configured distance telling the `TB3` to stop and call the `Niryo`. When the `Niryo` is called, it will start the pick-and-place program. After finishing it will signal to the `TB3` to continue his path to arrive at another aruco tag marking the end of the job.

### Running the program
Open a terminal for the `TB3`

```bash
ssh ubuntu@192.168.0.200

roslaunch turtlebot3_bringup turtlebot3_robot.launch 

roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch
```

Open a terminal for the `Niryo`

```bash
ssh niryo@192.168.0.150
```

Open a new terminal for the `Remote Pc` (every command in a different terminal tab)

```bash
roscore

roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action

roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action

rosrun turtlebot3_autorace_driving aruco.py

rosrun turtlebot3_autorace_driving niryo.py

roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

Or simply run 

```bash
gnome-terminal --tab --title="roscore" --command="bash -c 'roscore; exec bash'" \
               --tab --title="intrinsic calibration" --command="bash -c 'roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action; exec bash'" \
               --tab --title="extrinsic calibration" --command="bash -c 'roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action; exec bash'" \
               --tab --title="detect lane" --command="bash -c 'roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action; exec bash'" \
               --tab --title="aruco.py" --command="bash -c 'rosrun turtlebot3_autorace_driving aruco.py; exec bash'" \
               --tab --title="niryo.py" --command="bash -c 'rosrun turtlebot3_autorace_driving niryo.py; exec bash'" \
               --tab --title="drive tb3" --command="bash -c 'roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch; exec bash'" \
```



## Demo Videos
The below videos represent some tests on the TB3 autonomous drive in a dark and light environment, in addition the the Niryo pick-and-place task.


| **Dark Environment** | **Light Environment** |
| -------------------- | --------------------- |
| ![Dark Autonomous](images/dark_autonomous.gif) | ![Light Autonomous](images/light_autonomous.gif) |


| **Pick and Place** |
| ------------------ |
| ![Pick and Place](images/pick_n_place.gif) |


| **Final Result** |
| ---------------- |
| ![Pick and Place](images/final_result.gif) |


[1]:https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving
