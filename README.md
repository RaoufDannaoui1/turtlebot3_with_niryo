# Autonomous Robotic Coordination with TurtleBot3 and Niryo Ned2

Authors:
 - Abdel Raouf Dannaoui 
 - Nouriddin Asfour

Supervisors:
 - Joaquin Jorge Rodriguez
 - Raphael Duverne

![Cover](images/main.png)




## Project Goal
<b>Experience seamless robotic collaboration with this project!</b><br>

Our TurtleBot3 autonomously navigates between yellow and white lines based on [Autorace Challenge][1], pausing when detecting an ArUco tag at a specific distance. Once halted, it signals Niryo to execute a precise pick-and-place task. After completion, TurtleBot3 resumes its journey until it encounters another ArUco tag, marking the end of the simulation. Dive into the world of robotic coordination for efficient and intelligent task execution.




## Project Workflow 
<div style="text-align: center">
  <img src="images/flowchart.png"/>
</div>




## Required Libraries and Packages to Start
      ros-noetic-image-transport 
      ros-noetic-cv-bridge 
      ros-noetic-vision-opencv 
      python3-opencv 
      libopencv-dev 
      ros-noetic-image-proc
      pyniryo
      pyniryo2




## Installing the Required Packages.
For this step we have followed the instructions provided on [emanual robotics][1], and then manipulated those packages to match our project goal. Sstart by installing the edited AutoRace 2020 meta package on your Remote PC.

      cd ~/catkin_ws/src/
      git clone https://github.com/RaoufDannaoui1/turtlebot3_with_niryo.git
      cd ~/catkin_ws && catkin_make
            
Install additional dependent packages on Remote PC.

      sudo apt install ros-noetic-image-transport ros-noetic-cv-bridge ros-noetic-vision-opencv python3-opencv libopencv-dev ros-noetic-image-proc
      pip install pyniryo
      pip install pyniryo2




## TurtleBot3 Configuration
### Connection
Connect to the turtlebot3 thru the SSH on the specific IP and using `napelturbot` as password

    ssh ubuntu@192.168.0.200

Once connected, do not forget to gring up your robot using

     roslaunch turtlebot3_bringup turtlebot3_robot.launch 




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


[1]:https://emanual.robotis.com/docs/en/platform/turtlebot3/autonomous_driving/#autonomous-driving
