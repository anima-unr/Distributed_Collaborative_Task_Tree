#! /bin/bash
<<"COMMENT"
A simple shell script to run the multi_robot_simulator.
By executing the script it starts up multiple gnome-terminals, useful for debugging.

You may need to modify the startup function to cd to your proper workspace
COMMENT

function startup {
	cd ~/workspace
	source devel/setup.bash
}
function getTOTableTaskSim {
	cd src/Distributed_Collaborative_Task_Tree/table_task_sim/launch
}
function getTOMultiSim {
	cd src/Distributed_Collaborative_Task_Tree/multi_robot_sim/launch
}
function getTOPeerConnection {
	cd src/Distributed_Collaborative_Task_Tree/unr_object_manipulation/baxter_table_setting/launch
}
function getTOMutex {
	cd src/Distributed_Collaborative_Task_Tree/remote_mutex/launch
}
function baxter_startup {
	startup
	export ROS_MASTER_URI=http://${HOSTNAME}:1234
}
function pr2_activate {
rostopic pub /THEN_0_0_001_parent robotics_task_tree_msgs/ControlMessage "sender: {type: 0, robot: 0, node: 0}
type: 0
activation_level: 1000000000000.0
activation_potential: 0.0
done: false
active: false
highest: {type: 0, robot: 0, node: 0}
parent_type: 0"
}
#baxter_activate does not work with current version
function baxter_activate {
rostopic pub /AND_2_1_014_parent robotics_task_tree_msgs/ControlMessage "sender: {type: 0, robot: 0, node: 0}
type: 0
activation_level: 50000000000000000000000.0
activation_potential: 0.0
done: false
active: false
highest: {type: 0, robot: 0, node: 0}
parent_type: 0"
}
function getToDialogue {
	cd src/Distributed_Collaborative_Task_Tree/dialogue/launch
}
export -f startup getTOMultiSim getTOTableTaskSim getTOPeerConnection getTOMutex baxter_startup pr2_activate baxter_activate getToDialogue


#PR2 Setup
gnome-terminal -x bash -c "startup; roscore"
sleep .5
gnome-terminal -x bash -c "startup; getTOTableTaskSim; roslaunch table_task_sim.launch"
gnome-terminal -x bash -c "startup; getTOTableTaskSim; roslaunch dummy_multi_demo.launch"
gnome-terminal -x bash -c "startup; getTOPeerConnection; roslaunch peer_connection.launch"
gnome-terminal -x bash -c "startup; getTOMutex; roslaunch table_setting_mutex.launch"
#Baxter Setup
#gnome-terminal -x bash -c "baxter_startup; roscore -p 1234"
#sleep .5
#gnome-terminal -x bash -c "baxter_startup; getTOTableTaskSim; roslaunch table_task_sim.launch"
#gnome-terminal -x bash -c "baxter_startup; getTOTableTaskSim; roslaunch dummy_multi_demo_baxter.launch"
#gnome-terminal -x bash -c "baxter_startup; getTOPeerConnection; roslaunch peer_connection_baxter.launch"
#gnome-terminal -x bash -c "baxter_startup; getTOMutex; roslaunch table_setting_mutex_baxter.launch"
#Watcher
gnome-terminal -x bash -c "startup; getToDialogue; roslaunch dialogue.launch"
#Start Activation
sleep .5
gnome-terminal -x bash -c "startup; pr2_activate"
#sleep 1
#gnome-terminal -x bash -c "baxter_startup; baxter_activate"
