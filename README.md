# 106a_Project
Final Project for 106a


Commands in Order
1) open terminal and get roscore running
2) get realsense running using following command
   roslaunch realsense2_camera rs_camera.launch mode:=Manual color_width:=424 \
color_height:=240 depth_width:=424 depth_height:=240 align_depth:=true \
depth_fps:=6 color_fps:=6
3) run python vision script using: rosrun vision block_detector.py
4) open rviz with: rviz rviz
