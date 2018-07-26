
Instructions to test the human "robot" brain in the architecture:

#================================

#-----------
To test that the task allocation architecture is working for the human motions, run the following:
#-----------

Terminal 3:
  roslaunch table_setting_demo human_multi_demo_baxter_visionManip.launch 


Terminal 4:
  roslaunch unr_object_manipulation peer_connection_human_baxter_visionManip.launch 


Terminal 5:
   /TH0.0

Terminal 6:
  rostopic pub /THEN_0_1_010_parent robotics_task_tree_msgs/ControlMesge "sender: {type: 0, robot: 0, node: 0}
    type: 0
    activation_level: 1000000000000.0
    activation_potential: 0.0
    done: false
    active: false
    highest: {type: 0, robot: 0, node: 0}
    parent_type: 0" 

  **NOTE: For running the rostopic pub command, instead of copying it directly, please type out the rostopic pub /THEN_0_1_010_parent and then tab complete until the message and its empty contents comes up. Then use the arrowkeys to backtrack to the activation_level and enter a very large number as shown above (doesn't have to be exactly the number shown).**



#================================

#-----------
To verify that the human "robot" brain is working correctly with the architecture, pay attention to the following:
#-----------

In Terminal 5, the mutex should be granting access (locking and unlocking) nodes as the human interacts with objects. As the trial runs, we should see the following in this terminal:

  Once the object is "started", there should be a message printed here saying "Mutex Locked - Granted Access: PLACE_3_1_0XX" 
  Once the object is "done", there should be a message printed here saying "Mutex Unlocked - Granted Access: PLACE_3_1_0XX" 
  
  Where the "_0XX" corresponds to the object topic of the object in the YAML file. For simplicity, the mapping between objects and topics are listed here:
    PLACE_3_1_012 = clock
    PLACE_3_1_013 = teddy_bear
    PLACE_3_1_014 = scissors
    PLACE_3_1_016 = cup
    PLACE_3_1_017 = sports_ball


#================================

#-----------
Process for tetsing the architecture once all above things are running...
#-----------

Once all the teriminals are running, go through and pick up the objects in a way so that the constraints for the task are met, and watch the mutex as described above to verify the architecture is doing the right thing.

The constraints essentially mean that you should grab things in the following way:

  1. clock, teddy_bear, and scissors can be grabbed in any order, but all three must be grabbed.
  2. After the three above objects have been grabbed, grab EITHER the sports_ball OR the cup BUT NOT BOTH. 
  
If the remote mutex (Terminal 5) shows all the correct "Locking" and "Unlocking" messages for the chosen 4 objects in the right order, then the architecture works correctly! :)


#================================
DONE!
#================================


