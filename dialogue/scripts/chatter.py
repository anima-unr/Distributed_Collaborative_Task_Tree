#!/usr/bin/env python
import time
import rospy
from dialogue.msg import Issue
from dialogue.msg import Resolution
from std_msgs.msg import String
'''
	getYesOrNo
		gets a yes or no responce from the keyboard
	args: 
		none
	returns:
		true: if input = "yes", "y", or the return key
		false: if responce = "no", or "n"
'''
def getYesOrNo():
	done = False
	while not done:
		responce = raw_input("(Y/n)?\n").lower()
		if responce == "yes" or responce =="y" or responce =="":
			return True
		elif responce== "no" or responce =="n":
			return False
		else:
			print "Invalid responce! Try again!"
'''
dialogue
	Responds to an issue topic. Through the keyboard a human and robot communicate about
	how to solve a problem the robot is facing. When a solution has been found a resolution
	msg is published.

args:
	data: Issue.msg
	pub: resolution publishing topic
returns:
	void
'''
def dialogue(data, pub):
	issue=data.issue
	obj= data.object
	msg= Resolution()
	msg.object=obj
	msg.robot_id=data.robot_id
	msg.method=None
	while(msg.method==None):
		if issue == "dropped":
			print "It appears that I %s the %s." % (issue, obj)
			print "Should I try picking and placing the %s again?" %(obj)
			if getYesOrNo():
				msg.method="robot_pick_and_place"
				print "Alright! I will try picking and placing the %s again!" % (obj)
			else:
				print "Okay. Then will you pick and place the %s?" % (obj)
				if getYesOrNo():
					msg.method="human_pick_and_place"
					print "Thank you for picking and placing the %s!" % (obj)
		elif issue =="missing":
			print "I can't find the %s. Will you put it in sight?" %(obj)
			if getYesOrNo():
				msg.method="human_fetch_object"
				print "Thank you for fetching the %s!" %(obj)
		elif issue =="ungraspable":
			print "I can't reach the %s. Will you put it within my reach?" %(obj)
			if getYesOrNo():
				msg.method="human_place_object"
				print "Thanks for placing the %s within my reach" %(obj)
		elif issue == "positioning":
			print "I'm not sure if I've correctly placed the %s. Will you help me position it?" %(obj)
			if getYesOrNo():
				print "Thanks! Are you done yet?"
				if getYesOrNo():
					print "Thanks!"
					msg.method="human_position_obj"

	pub.publish(msg)
'''
chatter
	initializes the resolution publisher, chatter node, and issues subscriber.
args:
	none
returns:
	none
'''
def chatter():

    pub = rospy.Publisher('resolution', Resolution, queue_size=10)
    rospy.init_node('chatter', anonymous=True)
    rospy.Subscriber('issues', Issue, dialogue, pub)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
#initializes the dialogue
if __name__ == '__main__':
    chatter()



