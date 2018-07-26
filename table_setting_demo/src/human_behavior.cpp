#include <geometry_msgs/Pose.h>
#include <robotics_task_tree_msgs/ObjStatus.h>
#include <human_behavior.h>

namespace task_net {
////////////////////////////////////////////////////////////////////////////////
// Human PLACE BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
HumanBehavior::HumanBehavior() {}
HumanBehavior::HumanBehavior(NodeId_t name, NodeList peers, NodeList children,
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
    ROS_INFO( "HumanBehavior: [%s] Object: [%s]", name_->topic.c_str(), object_.c_str() );
    robot_des_ = robot_des;

    obj_chance_ = 0;
    obj_started_ = false;
    obj_done_ = false;

    // subscribe to object status messages from Bashira's work
    //char topic[];
    std::string topic = std::string("/") + object + std::string( "_status");
    //sprintf(topic, "/%s_status", object_.c_str());
    obj_status_sub_ = local_.subscribe(topic.c_str(), 1000, &HumanBehavior::ObjStatusCallback, this );
}
HumanBehavior::~HumanBehavior() {}

void HumanBehavior::UpdateActivationPotential() {
  if( state_.done || parent_done_ || state_.peer_active || state_.peer_done )
  {
    ROS_DEBUG_THROTTLE_NAMED( 1, "HumanBehaviorTrace", "[%s]: State/Parent is done, so don't update activationpotential [%d|%d|%d|%d]", object_.c_str(), state_.done, parent_done_, state_.peer_active, state_.peer_done );
    state_.activation_potential = 0;
    return;
  }

  if( !IsActive() )
  {
    ROS_INFO_THROTTLE_NAMED( 1, "HumanBehaviorTrace", "[%s]: Not active, so don't update activationpotential", object_.c_str() );
    state_.activation_potential = 0;
    return;
  }

  ROS_DEBUG_NAMED("HumanBehaviorTrace", "HumanBehavior::UpdateActivationPotential was called: [%s]", object_.c_str() );


  state_.activation_potential = obj_chance_; //TODO check order of magnitude
  ROS_WARN_NAMED("HumanBehavior", "%s: activation_potential: [%f]", object_.c_str(), state_.activation_potential );



}
  

bool HumanBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  // return true;

  if( obj_started_ == 1) {
    return true;
  }
  else{
    return false;
  }
}

uint32_t HumanBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  // ControlMessagePtr_t msg(new ControlMessage_t);
  // msg->sender = mask_;
  // msg->activation_level = 1.0f;
  // msg->done = false;
}

void HumanBehavior::Work() {
  ROS_INFO("HumanBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));
  // PickAndPlace(object_, robot_des_);
  //   // while (!PickAndPlaceDone()) {
  //     //boost::this_thread::sleep(boost::posix_time::millisec(500));
  //       // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
  //   // }
  // mut_arm.Release();

  while(obj_done_ == 0){
  ROS_WARN("[%s]: HumanBehavior::Working!", name_->topic.c_str());

    continue;
  }
  mut_arm.Release();
  state_.done = true;
  ROS_INFO("[%s]: HumanBehavior::Work: Done!", name_->topic.c_str());
  //ROS_INFO("\t[%s]: HumanBehavior::MUTEX IS RELEASED!", name_->topic.c_str());
}

// void HumanBehavior::PickAndPlace(std::string object, ROBOT robot_des) {

//   // pick
//   table_task_sim::PickUpObject req_pick;
//   req_pick.request.robot_id = (int)robot_des;
//   req_pick.request.object_name = object;
//   // table_task_sim::PickUpObject::Response res_pick; //to know if it failed or not...

//   // place
//   geometry_msgs::Pose pose;
//   pose.position.x = -0.45;
//   pose.position.y = 0.0;
//   pose.position.z = 0;
//   pose.orientation.x = 0;
//   pose.orientation.y = 0;
//   pose.orientation.z = 0;
//   pose.orientation.w = 1;

//   table_task_sim::PlaceObject req_place;
//   req_place.request.robot_id = (int)robot_des;
//   req_place.request.goal = pose;
//   // table_task_sim::PlaceObject::Response res_place; //to know if it failed or not...

//   // call the pick service
//   if(ros::service::call("pick_service", req_pick)) {

//     ROS_INFO("\t\t[%s]: THE PICK SERVICE WAS CALLED!!", name_->topic.c_str());

//     // call the place service
//     if(ros::service::call("place_service", req_place)) {
//       ROS_INFO("\t\t[%s]: THE PLACE SERVICE WAS CALLED!!", name_->topic.c_str());
//     }
//   }

//   state_.done = true;
//   ROS_INFO( "[%s]: PickAndPlace: everything is done", name_->topic.c_str() );

// }

bool HumanBehavior::PickAndPlaceDone() {
  // table_setting_demo::pick_and_place msg;
  // msg.request.object = object_;
  // ros::service::call("pick_and_place_check", msg);
  // return msg.response.success;
}

bool HumanBehavior::ActivationPrecondition() {
  ROS_DEBUG_NAMED("HumanBehaviorTrace", "\t[%s]: HumanBehavior::MUTEX IS LOCKING!", name_->topic.c_str());

  // orig
  // return mut_arm.Lock(state_.activation_potential);


  bool lock_okay;

 // first attempt which did not work.....
  // check peer states...?
  if(!state_.peer_active && !state_.peer_done) {
    // return mut_arm.Lock(state_.activation_potential);
    ROS_INFO("\t[%s]: Trying to gain access to mutex!", name_->topic.c_str());
    lock_okay = mut_arm.Lock(state_.activation_potential);
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


  void HumanBehavior::ObjStatusCallback( robotics_task_tree_msgs::ObjStatus msg)
  {
    // table_state_ = msg;
    obj_chance_ = msg.chance;
    obj_started_ = msg.started;
    obj_done_ = msg.done;

  }

}  // namespace task_net


