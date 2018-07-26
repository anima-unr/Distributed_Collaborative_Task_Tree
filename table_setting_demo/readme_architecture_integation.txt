roslaunch table_setting_demo multi_robot_task_demo_visionManip.launch 

to test objects:
  rosservice call /pick_and_place_object "teddy_bear"

 roslaunch table_setting_demo multi_robot_task_demo_visionManip.launch 
 roslaunch remote_mutex table_setting_mutex.launch 
 roslaunch unr_object_manipulation peer_connection_visionManip.launch 
 
 rostopic pub /AND_2_0_006_parent robotics_task_tree_msgs/ControlMessage "sender: {type: 0, robot: 0, node: 0}
type: 0
activation_level: 100000000.0
activation_potential: 0.0
done: false
active: false
highest: {type: 0, robot: 0, node: 0}
parent_type: 0" 
publishing and latching message. Press ctrl-C to terminate


