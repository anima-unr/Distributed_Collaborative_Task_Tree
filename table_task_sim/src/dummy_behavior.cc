#include <geometry_msgs/Pose.h>
#include <table_task_sim/PickUpObject.h>
#include <table_task_sim/PlaceObject.h>
#include <table_task_sim/dummy_behavior.h>

namespace task_net {
////////////////////////////////////////////////////////////////////////////////
// DUMMY PLACE BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
DummyBehavior::DummyBehavior() {}
DummyBehavior::DummyBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    std::string object,
    ROBOT robot_des,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state), mut_arm(name.topic.c_str(), "/right_arm_mutex") {

    object_ = object;
    robot_des_ = robot_des;

    // subscribe to simulator state messages
    state_sub_ = local_.subscribe("/state", 1000, &DummyBehavior::StateCallback, this );
}
DummyBehavior::~DummyBehavior() {}

void DummyBehavior::UpdateActivationPotential() {
    // ROS_INFO("AndBehavior::UpdateActivationPotential was called!!!!\n");

  state_.activation_potential = 100;
}

bool DummyBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  return true;
}

uint32_t DummyBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  // ControlMessagePtr_t msg(new ControlMessage_t);
  // msg->sender = mask_;
  // msg->activation_level = 1.0f;
  // msg->done = false;
}

void DummyBehavior::Work() {
  ROS_INFO("DummyBehavior::Work: waiting for pause to be done!");
  // boost::this_thread::sleep(boost::posix_time::millisec(10000));

  PickAndPlace(object_, robot_des_);
    // while (!PickAndPlaceDone()) {
      boost::this_thread::sleep(boost::posix_time::millisec(500));
        // ROS_INFO("TableObject::Work: waiting for pick and place to be done!");
    // }
  mut_arm.Release();
  ROS_INFO("DummyBehavior::Work: Done!");
  ROS_INFO("\tDmmyBehavior::MUTEX IS RELEASED!");
}

void DummyBehavior::PickAndPlace(std::string object, ROBOT robot_des) {

  // pick
  table_task_sim::PickUpObject req_pick;
  req_pick.request.robot_id = (int)robot_des;
  req_pick.request.object_name = object;
  // table_task_sim::PickUpObject::Response res_pick; //to know if it failed or not...

  // place
  geometry_msgs::Pose pose;
  pose.position.x = -0.25;
  pose.position.y = -.025;
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
  if(ros::service::call("pick_service", req_pick)) {

    ROS_INFO("\n\n\n\t\t THE PICK SERVICE WAS CALLED!!");

    // call the place service
    if(ros::service::call("place_service", req_place)) {
      ROS_INFO("\n\n\t\t THE PLACE SERVICE WAS CALLED!!\n\n\n");
    }
  }

}

bool DummyBehavior::PickAndPlaceDone() {
  // table_setting_demo::pick_and_place msg;
  // msg.request.object = object_;
  // ros::service::call("pick_and_place_check", msg);
  // return msg.response.success;
}

bool DummyBehavior::ActivationPrecondition() {
  ROS_INFO("\tDmmyBehavior::MUTEX IS LOCKING!");

  return mut_arm.Lock(state_.activation_potential);
  // return true;
}

  void DummyBehavior::StateCallback( table_task_sim::SimState msg)
  {
    table_state_ = msg;
  }

}  // namespace task_net
