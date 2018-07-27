#------------------------------------------
If working on PR2:
#------------------------------------------

Need to do following in each terminal:
 
 export ROS_MASTER_URI=http://prada:11311
 export ROS_IP=:10.68.###.##      (for your IP through VPN)
 source ~/onr_ws/devel/setup.bach (or your workspace for this code)
 
Then run the following (one new terminal per command): 
 
 cd /etc/ros/; roslaunch openni_head.launch
 rosrun rviz rviz
 roslaunch table_setting_demo new_test.launch
 roslaunch vision_manip_low_traffic.launch
 rosrun vision_manip_pipeline orthoProj
 roslaunch darknet_ros darknet_ros.launch 
 
 rosrun vision_manip_pipeline jb_yolo_obj_det_server.py
 rosrun vision_manip_pipeline jb_conv_coord_server
 rosrun vision_manip_pipeline jb_get_grasp_server.py 
 rosrun vision_manip_pipeline jb_pub_workspace_corners_server.py 
 rosrun vision_manip_pipeline jb_vision_manip_pipeline.py <object>
 
#------------------------------------------
If working on local Kinect:
#------------------------------------------

Run the following (one new terminal per command):

 roslaunch freenect_launch freenect.launch  depth_registration:=true
 rosrun rviz rviz
 roslaunch table_setting_demo new_test.launch
 roslaunch vision_manip_low_traffic.launch
 rosrun vision_manip_pipeline orthoProj

 rosrun vision_manip_pipeline jb_yolo_obj_det_server.py
 rosrun vision_manip_pipeline jb_conv_coord_server
 rosrun vision_manip_pipeline jb_get_grasp_server.py 
 rosrun vision_manip_pipeline jb_pub_workspace_corners_server.py 
 rosrun vision_manip_pipeline jb_vision_manip_pipeline.py <object>
 
 
