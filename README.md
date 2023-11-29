## Project Workflow 
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
