# Autonomous Robotic Coordination with TurtleBot3 and Niryo Ned2

Submited by:
 - Abdel Raouf Dannaoui 
 - Nouriddin Asfour

<br>

Supervised by:
 - Joaquin Jorge Rodriguez
 - Raphael Duverne
## 

## Project Goal
<b>Experience seamless robotic collaboration with this project!</b><br>
Our TurtleBot3 autonomously navigates between yellow and white lines, pausing when detecting an ArUco tag at a specific distance. Once halted, it signals Niryo to execute a precise pick-and-place task. After completion, TurtleBot3 resumes its journey until it encounters another ArUco tag, marking the end of the simulation. Dive into the world of robotic coordination for efficient and intelligent task execution.

## Project Workflow 
<p float="middle">
  <img src="images/flowchart.png"/>
</p>


      TB3
        |__camera_calibration
        |                   |__intrinsic_camera_calibration
        |                   |__extrinsic_camera_calibration
        |
        |__lane_detection
        |              |__calibration
        |              |           |__white_lane
        |              |           |__yellow_lane
        |              |
        |              |__action
        |                      |__lane_detection
        |__aruco
        |      |__marker_detection
        |                        |__pub_tb3_marker_distance 
        |
        |__drive
               |__pub_tb3_stop_status
               |__sub_N_pick_status

      Niryo
        |__sub_tb3_stop_status
        |
        |__pick_n_place
        |
        |__pub_N_pick_status
                          


## Demo and Trial Videos

Dark Enviroment.

<p float="middle">
  <img src="images/dark_autonomous.gif"/>
</p>


Light Enviroment.

<p float="middle">
  <img src="images/light_autonomous.gif"/>
</p>


Pick and Place.

<p float="middle">
  <img src="images/pick_n_place.gif"/>
</p>


Final Demo

<p float="middle">
  <img src="images/finalVideo.gif"/>
</p>
