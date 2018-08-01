#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <table_task_sim/Object.h>
#include <table_task_sim/Robot.h>
#include <table_task_sim/SimState.h>
#include "std_msgs/String.h"
#include <dialogue/Issue.h>
#include <dialogue/Resolution.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <table_task_sim/DropObject.h>
#include <table_task_sim/Position.h>
#include <table_task_sim/Vision.h>
#include <geometry_msgs/Pose.h>
#include <yaml-cpp/yaml.h>

// used to make sure that markers for robots/goals/objects do not collide and overwrite each other
#define OBJ_PFX 1000
#define ROB_PFX 2000
#define GOAL_PFX 3000

// defines top speed of robot TODO: define in a file
#define TOP_SPEED 0.1

// filename for yaml config file
table_task_sim::SimState load_state_from_file(std::string filename);
bool dialogueBit = false;
// message object used to store current sim state (published each simulator time_step)
table_task_sim::SimState simstate;
ros::Publisher vision_pub;
ros::Publisher position_pub;
/**
	lookup_object_by_name
		finds index of object in simstate by name
		publishes vision msgs that contain data on object position and index
		This is used as a pseudo vision system for the simulation.
	args:
		objname: name of object to find
		robot_id: robot_id in array found in simstate
	returns:
		index of object if found
		-1 if object not found

**/

int lookup_object_by_name( std::string objname, int robot_id)
{	
	table_task_sim::Vision msg;
	msg.robot_id=robot_id;
	msg.object=objname;

	//std::cout<<"Looked up "<<objname<<std::endl;
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		if ( objname.compare( simstate.objects[i].name ) == 0 )
		{
			//object found
			if(simstate.objects[i].visible)
			{
				msg.pose = simstate.objects[i].pose;
				msg.idx = i;
				vision_pub.publish(msg);
				return i;
			}

		}
	}
	// got through entire object list and did not find object	
	msg.idx = -1;
	vision_pub.publish(msg);
	return -1;
}

/**
	pick
		implements service PickUpObject.srv
		moves robot to object's location and then sets the robot as holding the object
		fails if object with name in req does not exist
		blocks until object is held (or failure)
		moves robot as long as dialogue bit is not active
		robot continuously checks to make sure object is still in sight and reachable.

	args:
		req: robot id and object to pick up (name as string)
		res: result (0 if successful, 1 if failure)

	returns:
		true: if service was successfully called
		false: never
**/
bool pick(table_task_sim::PickUpObject::Request  &req,
          table_task_sim::PickUpObject::Response &res)
{
  //res.result = table_task_sim::PickUpObject::SUCCESS;
	res.result = 0;
	int idx;
	float dist = 999;
	ros::Rate loop_rate( 10 );

	// wait for robot to reach goal
	do {
		if(dialogueBit == false)
		{
			idx = lookup_object_by_name(req.object_name, req.robot_id);
			if(idx >= 0)
			{
				simstate.robots[req.robot_id].goal = simstate.objects[idx].pose;
				float xdist = simstate.robots[req.robot_id].goal.position.x - simstate.robots[req.robot_id].pose.position.x;
				float ydist = simstate.robots[req.robot_id].goal.position.y - simstate.robots[req.robot_id].pose.position.y;
				dist = hypot(ydist,xdist);

			}
		}
		//ROS_INFO ("robot [%d] moving to [%s] dist: %f", req.robot_id, req.object_name.c_str(), dist);
		loop_rate.sleep();
	} while( dist > 0.0001 );

	simstate.robots[req.robot_id].holding = req.object_name;

	res.result = 0;
	return true;
}

/**
	place
		implements service PlaceObject.srv
		moves robot and held object to goal location and then stops holding object
		fails if robot is not holding an object
		blocks until goal is reached (or failure)
		movement is paused when the dialogue bit is activated
		when the object reaches its goal location there is another check to make sure the object is placed correctly

	args:
		req: robot id and goal location (Pose)
		res: result (0 if successful, 1 if failure)

	returns:
		true: if service was successfully called
		false: never
**/


bool place(table_task_sim::PlaceObject::Request		&req,
		   table_task_sim::PlaceObject::Response	&res)
{
	if( simstate.robots[req.robot_id].holding.compare("") == 0 )
	{
		// not holding anything
		res.result = 1;
		return true;
	}
	// move to place goal
	simstate.robots[req.robot_id].goal = req.goal;
	float dist = 999;
	ros::Rate loop_rate( 10 );

	// wait for robot to reach goal
	do {

			float xdist = simstate.robots[req.robot_id].goal.position.x - simstate.robots[req.robot_id].pose.position.x;
			float ydist = simstate.robots[req.robot_id].goal.position.y - simstate.robots[req.robot_id].pose.position.y;
			dist = hypot(ydist,xdist);
			std::cout<<dist<<std::endl;
			loop_rate.sleep();
		} while( dist > 0.0001 );
	// drop object
	table_task_sim::Position msg;
	msg.obj=simstate.robots[req.robot_id].holding;
	msg.robot_id= req.robot_id;
	position_pub.publish(msg);
	while(dialogueBit)
	{
		loop_rate.sleep();
	}
	simstate.robots[req.robot_id].holding = std::string("");
	
	res.result = 0;
	return true;
}
/*
NOT SURE IF SHOULD KEEP
bool removeObject(table_task_sim::RemoveObject::Request &req,
					table_task_sim::RemoveObject::Response &res)
{
	
	int idx=lookup_object_by_name(req.object_name);
	if(idx==-1)
	{
		res.result = -1;
		return true;
	}
	else
	{	
		//makes object invisible
		simstate.objects[idx].visible=false;

		res.result = 0;	
		std::cout<<"removed "<<req.object_name<<" "<<simstate.objects[idx].visible<<std::endl;
		return true;
	}
}
*/
/*
	human_fetch_object
		Makes an object that was hidden visible, 
		simulating a human fetching an object missing from the table
	args:
		string obj: name of the object
	returns:
		the index of the object, if not found then -1
*/
int human_fetch_object(std::string obj)
{
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		if ( obj.compare( simstate.objects[i].name ) == 0 )
		{
			//object is made visible
			simstate.objects[i].visible=true;
			return i;
		}
	}
	return -1;
}
/*
	drop
		implements service DropObject.srv
		clears the robot's holding string indicating the object has been dropped.
		fails if the robot is not holding an object.
	args:
		req: robot id
		res: result (0 if successful, 1 if failure)
	returns:
		true: if service was successfully called
		false: never
*/

bool drop(table_task_sim::DropObject::Request	&req,
		   table_task_sim::DropObject::Response	&res)
{
	if( simstate.robots[req.robot_id].holding.compare("") == 0 )
	{
		// not holding anything
		res.result = 1;
		return true;
	}

	std::cout<<"Dropped "<<simstate.robots[req.robot_id].holding<<std::endl<<std::endl;
	simstate.robots[req.robot_id].holding = std::string("");
	res.result= 0;
	return true;
}
/*
	robot_pick_and_place
		simulates the robot trying to pick up a dropped object again and continuing with its
		goal of correctly placing the object
	args:
		obj: object name
		robot_id: robot id (almost always 0)
	returns:
		void
*/
void robot_pick_and_place(std::string obj, int robot_id)
{
	std::cout<<obj<<robot_id<<std::endl;
	
	int idx = lookup_object_by_name(obj, robot_id);
	simstate.robots[robot_id].pose.position.x = simstate.objects[idx].pose.position.x;
	simstate.robots[robot_id].pose.position.y = simstate.objects[idx].pose.position.y;
	simstate.robots[robot_id].holding = obj;	
}
/*
	human_pick_and_place
		simulates a human picking and placing a dropped object and completing that portion of the task
	args:
		obj: object name
		robot_id: robot id
	returns:
		void
*/
void human_pick_and_place(std::string obj, int robot_id)
{
	int idx = lookup_object_by_name(obj, robot_id);
	simstate.objects[idx].pose.position.x = simstate.robots[robot_id].goal.position.x;
	simstate.objects[idx].pose.position.y = simstate.robots[robot_id].goal.position.y;
	simstate.robots[robot_id].goal.position = simstate.robots[robot_id].pose.position;

}
/*
	human_place_object
		simulates the human placing an object to a reachable location for the robot
	args:
		obj: object name
		robot_id: robot id
	returns:
		void
*/
void human_place_object(std::string obj, int robot_id)
{
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		if ( obj.compare( simstate.objects[i].name ) == 0 )
		{

			simstate.objects[i].pose.position.x = 0.0;
			simstate.objects[i].pose.position.y = 0.0;
			simstate.robots[robot_id].goal.position = simstate.objects[i].pose.position;
		}
	}
}
/*
	pause(msg)
		sets the dialogueBit to true thus pausing the architecture.
	args:
		dialogue::Issue msg
	returns:
		void
*/
void pause(const dialogue::Issue::ConstPtr& msg)
{
	dialogueBit = true;	
}
/*
	resume(msg)
		a callback function that recieve resolutions and tells the simulation how to act
		with respect to the robot and object.
		sets the dialogueBit to false.

	args:
		dialogue::Resolution msg

	returns:
		void
*/
void resume(const dialogue::Resolution::ConstPtr& msg)
{	
	std::string obj= msg->object.c_str();
	std::string method= msg->method.c_str();
	int robot_id= msg->robot_id;
	//responce to dropped object
	if(method == "robot_pick_and_place")
	{
		robot_pick_and_place(obj, robot_id);
	}
	//responce to dropped object
	else if(method == "human_pick_and_place")
	{
		human_pick_and_place(obj, robot_id);
	}
	//responce to object not found
	else if(method == "human_fetch_object")
	{
		human_fetch_object(obj);
		//tell pick to resume some how?
	}
	//responce to object being unreachable
	else if(method == "human_place_object")
	{
		human_place_object(obj, robot_id);
	}
	//responce to robot asking for help with positioning the object in its goal location
	else if(method == "human_position_object"){}

// Can't do this because the robot has to Pick then Place causing to go to the goal pos every time
	/*
	else if(method == "human_fetch_and_place")
	{
		int idx=human_fetch_object(obj);
		simstate.objects[idx].pose.position.x = -0.45;
		simstate.objects[idx].pose.position.y = 0.0;
		simstate.robots[robot_id].goal.position.x = simstate.robots[robot_id].pose.position.x;
		simstate.robots[robot_id].goal.position.y = simstate.robots[robot_id].pose.position.y;
		
	}
	*/
	dialogueBit = false;
}


/**
	publish_markers(mp)
		publishes markers for viewing simulator state in rviz
		creates marker message and adds table surface, robots, goals, and objects
		populates information from simstate variable

	args:
		mp: ros publisher (previously instantiated in main)

	returns:
		void
**/
void publish_markers(ros::Publisher *mp)
{
	visualization_msgs::Marker marker;

	// these values are the same for all
	marker.header.frame_id = "/table";
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;

	// publish table
	marker.id = 0001;
	marker.type = visualization_msgs::Marker::CUBE;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = -0.02;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 2.0;
    marker.scale.y = 1.0;
    marker.scale.z = 0.01;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.1f;
    marker.color.g = 0.1f;
    marker.color.b = 0.1f;
    marker.color.a = 1.0;

	mp->publish(marker);

	// publish object poses
	marker.type = visualization_msgs::Marker::CUBE;
	for( int i = 0; i < simstate.objects.size(); i++ )
	{
		//only publishes if the object is visible
		if(simstate.objects[i].visible)
		{
			marker.id = OBJ_PFX + i;
			marker.pose = simstate.objects[i].pose;
		    marker.scale = simstate.objects[i].scale;
		    marker.color = simstate.objects[i].color;
		   	mp->publish(marker);
	   }
	}

	// publish robot poses
	marker.type = visualization_msgs::Marker::CYLINDER;
	for( int i = 0; i < simstate.robots.size(); i++ )
	{
		marker.id = ROB_PFX + i;
		marker.pose = simstate.robots[i].pose;
	    marker.scale.x = 0.075;
	    marker.scale.y = 0.075;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;

		mp->publish(marker);		

		marker.id = GOAL_PFX + i;
		marker.pose = simstate.robots[i].goal;
	    marker.scale.x = 0.03;
	    marker.scale.y = 0.03;
	    marker.scale.z = 0.01;
	    marker.color = simstate.robots[i].color;
	    marker.color.a = 0.5;

		mp->publish(marker);		

	}
}

/**
	populate_state()
		function to populate the initial state variables
		below you can list which objects you want to initially hide/set to not visible.
		TODO: read from file or parameter service (probably file) instead of hard-coded values

	args: none
	returns: void
**/

void populate_state(std::string filename)
{
	YAML::Node config = YAML::LoadFile(filename.c_str());
	if( config.IsNull() )
	{
		ROS_ERROR( "file not found: [%s]", filename.c_str());
	}

	YAML::Node objects = config["objects"];
	for( int i = 0; i < objects.size(); i++ )
	{
		table_task_sim::Object obj;
		obj.name = objects[i]["name"].as<std::string>();
		ROS_INFO( "loading object: [%s]", obj.name.c_str() );
		obj.pose.position.x = objects[i]["location"]["x"].as<double>();
		obj.pose.position.y = objects[i]["location"]["y"].as<double>();
		obj.pose.position.z = 0.0;
		obj.pose.orientation.x = 0.0;
		obj.pose.orientation.y = 0.0;
		obj.pose.orientation.z = 0.0;
		obj.pose.orientation.w = 1.0;
		obj.scale.x = objects[i]["scale"]["x"].as<double>();
		obj.scale.y = objects[i]["scale"]["y"].as<double>();
		obj.scale.z = objects[i]["scale"]["z"].as<double>();
		obj.color.r = objects[i]["color"]["r"].as<double>();
		obj.color.g = objects[i]["color"]["g"].as<double>();
		obj.color.b = objects[i]["color"]["b"].as<double>();
		obj.color.a = objects[i]["color"]["a"].as<double>();
		//set objects to initially hide
		if( obj.name=="scissors" || obj.name=="teddy_bear" || obj.name == "clock")
		{
			obj.visible =false;
		}
		else
		{
			obj.visible = true;
		}
		//obj.visible= true;
		simstate.objects.push_back(obj);
	}

	YAML::Node robots = config["robots"];
	for( int i = 0; i < robots.size(); i++ )
	{
		table_task_sim::Robot rob;
		rob.pose.position.x = robots[i]["location"]["x"].as<double>();
		rob.pose.position.y = robots[i]["location"]["y"].as<double>();
		rob.pose.position.z = 0.0;
		rob.color.r = robots[i]["color"]["r"].as<double>();
		rob.color.g = robots[i]["color"]["g"].as<double>();
		rob.color.b = robots[i]["color"]["b"].as<double>();
		rob.color.a = robots[i]["color"]["a"].as<double>();
		rob.goal = rob.pose;

		simstate.robots.push_back(rob);
	}
}


int main(int argc, char* argv[] )
{
	ros::init(argc, argv, "table_sim");
	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");

	ros::Rate loop_rate(100);
	
	std::string filename;
	if( nh_priv.getParam("filename", filename) ) 
	{
		ROS_INFO( "simulator loading config from filename: [%s]", filename.c_str() );
		// populate data (TODO replace with file I/O)
		populate_state(filename);
	}
	else
	{
		ROS_INFO( "file not found... [%s]", filename.c_str() );
		return 1;
	}

	// declare subscribers
	ros::ServiceServer pick_service = nh.advertiseService("pick_service", pick);
	ros::ServiceServer place_service = nh.advertiseService("place_service", place);
	ros::ServiceServer drop_service = nh.advertiseService("drop_service", drop);
	//ros::ServiceServer remove_service = nh.advertiseService("remove_service", removeObject);
	
	ros::Subscriber issue_sub = nh.subscribe<dialogue::Issue>("issues", 1000, &pause);
	ros::Subscriber resolution_sub = nh.subscribe<dialogue::Resolution>("resolution", 1000, &resume);
	// declare publishers
	ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("markers", 1000);
	ros::Publisher state_pub = nh.advertise<table_task_sim::SimState>("state", 1000);
	vision_pub = nh.advertise<table_task_sim::Vision>("vision", 1000); 
	position_pub = nh.advertise<table_task_sim::Position>("position", 1000);
	// async spinner thread
	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();

	ros::Time last_iter = ros::Time::now();

	/* main control loop */
	while( ros::ok() )
	{
		ros::Time curr_time = ros::Time::now();

		// update end effector positions
		ros::Duration diff = curr_time - last_iter;
		for( int i = 0; i < simstate.robots.size(); i++ )
		{
			// get distance from current pos to goal
			float xdist = simstate.robots[i].goal.position.x - simstate.robots[i].pose.position.x;
			float ydist = simstate.robots[i].goal.position.y - simstate.robots[i].pose.position.y;
			float dist = hypot(ydist,xdist);
			float theta = atan2(ydist, xdist);

			float r = TOP_SPEED * diff.toSec();

			// if dist is less than top speed / dir
			if( dist < r )
			{
				// robot has reached goal in this iter, just set the current pos to the goal
				simstate.robots[i].pose = simstate.robots[i].goal;
			}
			//only move if the dialoge bit is not active
			else if(dialogueBit==false)
			{
				// move direction of travel top speed / duration
				simstate.robots[i].pose.position.x += r * cos(theta);
				simstate.robots[i].pose.position.y += r * sin(theta);
			}

			// if the robot is holding an object, move the object to where the robot is
			if( simstate.robots[i].holding.length() > 0 )
			{
				// find object with the name in holding
				int idx = lookup_object_by_name(simstate.robots[i].holding, i);
				simstate.objects[idx].pose.position = simstate.robots[i].pose.position;
			} 
		} // for i

		// publish markers
		publish_markers(&marker_pub);

		// publish current object and end effector positions 
		state_pub.publish(simstate);

		last_iter = curr_time;
		loop_rate.sleep();
	} // while ros::ok()

	return 0;
}