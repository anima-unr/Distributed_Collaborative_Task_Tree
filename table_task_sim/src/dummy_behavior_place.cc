#include <geometry_msgs/Pose.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <table_task_sim/dummy_behavior_place.h>
#include <table_task_sim/dummy_behavior_pick.h>

#include <table_task_sim/header.h>
namespace task_net {
////////////////////////////////////////////////////////////////////////////////
// DUMMY PLACE BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DummyBehaviorPlace::DummyBehaviorPlace() {}
DummyBehaviorPlace::DummyBehaviorPlace(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    ROBOT robot_des,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state ,
      object), mut_arm(name.topic.c_str(), "/right_arm_mutex") {

    object_ = object;
    state_.done = false;
    parent_done_ = false;
    
    robot_des_ = robot_des;

    // subscribe to simulator state messages
    state_sub_ = local_.subscribe("/state", 1000, &DummyBehaviorPlace::StateCallback, this );
}
DummyBehaviorPlace::~DummyBehaviorPlace() {}

void DummyBehaviorPlace::UpdateActivationPotential() {
  if( state_.done || parent_done_ || state_.peer_active || state_.peer_done )
  {
    ROS_DEBUG_THROTTLE_NAMED( 1, "DummyBehaviorPlaceTrace", "[%s]: State/Parent is done, so don't update activationpotential [%d|%d|%d|%d]", object_.c_str(), state_.done, parent_done_, state_.peer_active, state_.peer_done );
    state_.activation_potential = 0;
    return;
  }

  if( !IsActive() )
  {
    ROS_INFO_THROTTLE_NAMED( 1, "DummyBehaviorPlaceTrace", "[%s]: Not active, so don't update activationpotential_placeeeee", object_.c_str() );
    state_.activation_potential = 0;
    return;
  }

  ROS_DEBUG_NAMED("DummyBehaviorTracePlace", "DummyBehaviorPlace::UpdateActivationPotential was called: [%s]", object_.c_str() );

  geometry_msgs::Point rpos, opos;

  if( table_state_.robots.size() == 0 || table_state_.objects.size() == 0 )
  {
    ROS_WARN("state has not been populated, yet");
    return;
  }


  // get location of robot
  rpos = table_state_.robots[robot_des_].pose.position;

  // get location of object

    // find object
  int obj_idx = -1;
  for( int i = 0; i < table_state_.objects.size(); i++ )
  {
    //OS_INFO ("%s =?= %s",table_state_.objects[i].name.c_str(), object_.c_str() );
    if( table_state_.objects[i].name.compare(object_) == 0 )
    {
      obj_idx = i;
      break;
    }
  }

  if( obj_idx < 0 )
  {
    ROS_WARN( "could not find object: [%s]", object_.c_str() );
  }
  opos = table_state_.objects[obj_idx].pose.position;
  
  /*geometry_msgs::Pose pose;
  pose.position.x = -0.45;
  pose.position.y = 0.0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  double c1 = 3.0; // weight for distance
  double c2 = 1.0; //weight for suitability
  double dist = hypot(rpos.y -  pose.position.y, rpos.x -  pose.position.x);

  if( fabs(dist) > 0.00001 )
      state_.activation_potential = ( c1 * (1.0f / dist)) + (c2 * state_.suitability);
  else state_.activation_potential = 0.00000001;*/
 
  
  if((hold==1)  && (hold_object_name.compare(object_)==0)){
  	state_.activation_potential =1;
  	}
  	else{
  	state_.activation_potential=0;
  	}
 
}
  

bool DummyBehaviorPlace::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  return true;
}

uint32_t DummyBehaviorPlace::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  // ControlMessagePtr_t msg(new ControlMessage_t);
  // msg->sender = mask_;
  // msg->activation_level = 1.0f;
  // msg->done = false;
}

void DummyBehaviorPlace::Work() {
  ROS_INFO("DummyBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));
  Place(object_, robot_des_);
    // while (!PickAndPlaceDone()) {
      //boost::this_thread::sleep(boost::posix_time::millisec(500));
        // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
    // }
  mut_arm.Release();
 // if(mut_arm.Release()) {hold=0;hold_object_name="N/A";}
  ROS_INFO("[%s]: DummyBehavior::Work: Done!", name_->topic.c_str());
  hold=0;
 // hold_object_name="N/A";
  pick=0;
  //ROS_INFO("\t[%s]: DummyBehavior::MUTEX IS RELEASED!", name_->topic.c_str());
}

void DummyBehaviorPlace::Place(std::string object, ROBOT robot_des) {

  

  // place
  geometry_msgs::Pose pose;
  pose.position.x = -0.45;
  pose.position.y = 0.0;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;

  table_task_sim::PlaceObject req_place;
  req_place.request.robot_id = (int)robot_des;
  req_place.request.goal = pose;
  // table_task_sim::PlaceObject::Response res_place; //to know if it failed or not...

  // call the pick service
  //if(ros::service::call("pick_service", req_pick)) {

    //ROS_INFO("\t\t[%s]: THE PICK SERVICE WAS CALLED!!", name_->topic.c_str());

    // call the place service
    if(ros::service::call("place_service", req_place)) {
      ROS_INFO("\t\t[%s]: THE PLACE SERVICE WAS CALLED!!", name_->topic.c_str());
    }
  //}

  state_.done = true;
  
  //hold = 0;
  //hold_status_.object_name="N/A";
  ROS_INFO( "[%s]: PickAndPlace: everything is done", name_->topic.c_str() );

}

bool DummyBehaviorPlace::PlaceDone() {
  // table_setting_demo::pick_and_place msg;
  // msg.request.object = object_;
  // ros::service::call("pick_and_place_check", msg);
  // return msg.response.success;
}

bool DummyBehaviorPlace::ActivationPrecondition() {
  ROS_DEBUG_NAMED("DummyBehaviorTrace", "\t[%s]: DummyBehavior::MUTEX IS LOCKING!", name_->topic.c_str());

  // orig
  // return mut_arm.Lock(state_.activation_potential);


  bool lock_okay;

 // first attempt which did not work.....
  // check peer states...?
  if(!state_.peer_active && !state_.peer_done) {
    // return mut_arm.Lock(state_.activation_potential);
    ROS_INFO("\t[%s]: Trying to gain access to mutex!", name_->topic.c_str());
    lock_okay = mut_arm.Lock(state_.activation_potential);
    printf("%f activation potential of place %s\n",state_.activation_potential,name_->topic.c_str());
    // return true;
  }
  else{
    ROS_INFO("\t[%s]: Peer gained access first, don't activate!", name_->topic.c_str());
    return false;
  }

// second attempt
  // check peer states...?
  if(state_.peer_active || state_.peer_done) {
    ROS_INFO("\t[%s]: Peer gained access first so release mutex and don't activate!", name_->topic.c_str());
    mut_arm.Release();
    return false;
  }

  return lock_okay;
}

  void DummyBehaviorPlace::StateCallback( table_task_sim::SimState msg)
  {
    table_state_ = msg;
  }

}  // namespace task_net
