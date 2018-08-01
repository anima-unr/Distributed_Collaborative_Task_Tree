#!/usr/bin/env python
'''
The watcher.py is responsible for keeping track of proper placement of objects.
When the watcher realizes something goes wrong it publishes it to the issues topic.
'''
import rospy
import math
from table_task_sim.msg import Object, Robot, SimState, Vision, Position
from dialogue.msg import Issue
from std_msgs.msg import String
prevholding=['']
#TODO: Make it work for two robots currently only works for one robot.
'''
    dropCheck
        This keeps track of whether the robot has dropped an object
        When the object is dropped an Issue msg is published
    args:
        data: msgs from the state topic
        pub: issues publisher
    returns:
        void
'''
def dropCheck(data, pub):
    robot_pos=data.robots[0].pose.position
    goal_pos=data.robots[0].goal.position
    holding=data.robots[0].holding
    if prevholding[0] and not holding and robot_pos!=goal_pos:
                msg= Issue()
                msg.issue = "dropped"
                msg.object = prevholding[0]
                pub.publish(msg)
                rospy.loginfo(msg.issue)
    prevholding[0]= holding
'''
    positionCheck
        checks against a list of objects to see if a human should help the robot in positioning them
        if so an issue msg is published
    args:
        data: msgs from the position topic
        pub: issues publisher
    returns:
        void
'''
def positionCheck(data, pub):
    objects=['apple','cup','scissors', 'teddy_bear', 'clock']
    if data.obj in objects:
        print "pos check"
        msg = Issue()
        msg.issue = "positioning"
        msg.object= data.obj
        pub.publish(msg)
'''
    visionCheck
        checks to make sure an object is in sight and reachable
        if either are not satisfied an issue msg is published
    args:
        data: msgs from the vision topic
        pub: issues publisher
    returns:
        void
'''
def visionCheck(data, pub):
    msg= Issue()
    msg.object = data.object
    msg.robot_id= data.robot_id
    if data.idx < 0:
        msg.issue = "missing"
        pub.publish(msg)
    #grasp check
    else:
        robot_x = 0.0
        robot_y = -0.45
        if data.robot_id != 0:
            robot_y= 0.45
        xdist=data.pose.position.x - robot_x
        ydist=data.pose.position.y - robot_y
        dist= math.hypot(xdist,ydist)
        if dist > 0.68:
            msg.issue = "ungraspable"
            pub.publish(msg)


'''
    The watcher publishs to the issues topic which notifies the main update found in node.cc
    that something went wrong. This is currently set up for the simulator since it subscibes to the state topic.
'''
def watcher():

    pub = rospy.Publisher('issues', Issue, queue_size=10)
    rospy.init_node('watcher', anonymous=True)
    rospy.Subscriber('state', SimState, dropCheck, pub)
    rospy.Subscriber('vision', Vision, visionCheck, pub)
    rospy.Subscriber('position', Position, positionCheck, pub)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
#initializes the watcher
if __name__ == '__main__':
    watcher()
