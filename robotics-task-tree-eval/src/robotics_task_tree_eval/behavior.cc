/*
robotics-task-tree-eval
Copyright (C) 2015  Luke Fraser

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include "robotics_task_tree_eval/behavior.h"
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include "remote_mutex/remote_mutex.h"
//#include "geometry_msgs/Pose.h"

namespace task_net {
typedef std::vector<NodeId_t>::iterator NodeId_t_iterator;
// BEHAVIOR
Behavior::Behavior() {}
Behavior::Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Node(name,
      peers,
      children,
      parent,
      state) {  
      // printf("Behavior::Behavior WAS CALLED\n");
}
Behavior::~Behavior() {}

////////////////////////////////////////////////////////////////////////////////
// AND BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
AndBehavior::AndBehavior() {}
AndBehavior::AndBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
        // printf("AndBehavior::AndBehavior WAS CALLED\n");
    }
AndBehavior::~AndBehavior() {}

void AndBehavior::UpdateActivationPotential() {
    // ROS_INFO("AndBehavior::UpdateActivationPotential was called!!!!\n");

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  // this should choose bubble up child with highest potential
  float highest = -1;
  NodeBitmask nbm = mask_;

  float sum = 0;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    sum += (*it)->state.activation_potential;
    if( (*it)->state.activation_potential > highest && !(*it)->state.done && !(*it)->state.peer_done && !(*it)->state.peer_active && !(*it)->state.active )
    {
      // save as the highest potential
      highest = (*it)->state.activation_potential;
      nbm = (*it)->mask;
    }
  }
  state_.activation_potential = sum / children_.size();
  state_.highest_potential = highest;
  state_.highest = nbm;
}

bool AndBehavior::Precondition() {
    // ROS_INFO("AndBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t AndBehavior::SpreadActivation() {
    // ROS_INFO("AndBehavior::SpreadActivation was called!!!!");
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->type = 0;
  msg->sender = mask_;
  msg->activation_level = 1.0f / children_.size();
  msg->done = false;

  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    SendToChild((*it)->mask, msg);
  }
}

bool AndBehavior::IsDone() {
  ROS_DEBUG("[%s]: AndBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( !(children_[i]->state.done || children_[i]->state.peer_done) ) 
    {
      ROS_DEBUG( "[%s]: state not done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 0;
      return false;
    }
  }

  state_.done = 1;
  return true;
  //return state_.done;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// THEN BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
ThenBehavior::ThenBehavior() {}
ThenBehavior::ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  // Initialize activation queue
      // printf("ThenBehavior::ThenBehavior WAS CALLED\n");
  for (NodeListPtrIterator it = children_.begin(); it != children_.end();
      ++it) {
    activation_queue_.push(*it);
  }
}
ThenBehavior::~ThenBehavior() {}

void ThenBehavior::UpdateActivationPotential() {
  //ROS_INFO("ThenBehavior::UpdateActivationPotential was called!!!!\n");

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  // this should bubble up first not done child
  float highest = 0;
  NodeBitmask nbm = mask_;


  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    //ROS_INFO( "\t[%s]: checking [%d]: [%d|%d|%d|%d]", name_->topic.c_str(), (*it)->state.owner.node, (*it)->state.done, (*it)->state.peer_done, (*it)->state.active, (*it)->state.peer_active );
    // find the first not done/active node
    if( !(*it)->state.done && !(*it)->state.peer_done && !(*it)->state.peer_active )
    {
      // save as the highest potential
      highest = (*it)->state.activation_potential;
      nbm = (*it)->mask;
      break;
    }
  }

  state_.activation_potential = highest; //sum / children_.size();
  state_.highest_potential = highest;
  state_.highest = nbm;

}

bool ThenBehavior::Precondition() {
    // ROS_INFO("ThenBehavior::Precondition was called!!!!\n");
  bool satisfied = true;
  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    satisfied = satisfied && (*it)->state.done;
  }
  if (satisfied)
    return true;
  return false;
}

uint32_t ThenBehavior::SpreadActivation() {
  ROS_DEBUG("ThenBehavior::SpreadActivation was called!!!!");
  if (!activation_queue_.empty()) {
    ControlMessagePtr_t msg(new ControlMessage_t);
    msg->type = 0;
    msg->sender = mask_;
    msg->activation_level = 1.0f;
    msg->done = false;

    if (activation_queue_.front()->state.done || activation_queue_.front()->state.peer_done) {
      int old_id = activation_queue_.front()->state.owner.node;
      activation_queue_.pop();
      ROS_INFO( "\t[%s]: node [%d] is done, moving to next node [%d]", name_->topic.c_str(), old_id, activation_queue_.front()->state.owner.node);
    }

    ROS_DEBUG( "[%s]: spreading activation to [%d]", name_->topic.c_str(), activation_queue_.front()->state.owner.node );
    SendToChild(activation_queue_.front()->mask, msg);
  }
}

bool ThenBehavior::IsDone() {
  ROS_DEBUG("[%s]: ThenBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( !(children_[i]->state.done || children_[i]->state.peer_done) ) 
    {
      ROS_DEBUG( "[%s]: state not done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 0;
      return false;
    }
  }

  state_.done = 1;
  return true;
  //return state_.done;
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// OR BEHAVIOR
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
OrBehavior::OrBehavior() {}
OrBehavior::OrBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue,
    boost::posix_time::millisec mtime) : Behavior(name,
      peers,
      children,
      parent,
      state) {
  seed = static_cast<uint32_t>(time(NULL));
  //random_child_selection = rand_r(&seed) % children_.size();
  // printf("OrBehavior::OrBehavior WAS CALLED\n");
}
OrBehavior::~OrBehavior() {}

void OrBehavior::UpdateActivationPotential() {
    // ROS_INFO("OrBehavior::UpdateActivationPotential was called!!!!\n");
  float max = 0;
  int max_child_index = 0, index = 0;

  // this should choose bubble up child with highest potential
  NodeBitmask nbm = mask_;

  // if node is done, activation potential for all children is 0
  if( IsDone() )
  {
    state_.highest_potential = 0;
    state_.highest = mask_;
    return;
  }

  for (NodeListPtrIterator it = children_.begin();
      it != children_.end(); ++it) {
    float value = (*it)->state.activation_potential;
    if (value > max && !(*it)->state.done && !(*it)->state.peer_done && !(*it)->state.peer_active && !(*it)->state.active) {
      max = value;
      max_child_index = index;
      nbm = (*it)->mask;
    }
    index++;
  }
  state_.activation_potential = max;
  state_.highest_potential = max;
  state_.highest = nbm;
  //random_child_selection = max_child_index;
}

bool OrBehavior::Precondition() {
  ROS_DEBUG("OrBehavior::Precondition was called!!!!\n");
  
  for( int i = 0; i < children_.size(); i++ )
  {
    if( children_[i]->state.done ) 
    {
      ROS_INFO( "[%s]: state done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 1;
      return true;
    }
  }

  return false;
}

uint32_t OrBehavior::SpreadActivation() {
  ROS_DEBUG("OrBehavior::SpreadActivation was called!!!!");
  ControlMessagePtr_t msg(new ControlMessage_t);
  msg->type = 0;
  msg->sender = mask_;
  msg->activation_level = 1.0f;
  msg->done = false;

  for( int i = 0; i < children_.size(); i++ )
  {
    SendToChild(children_[i]->mask, msg);  
  }
  
}

bool OrBehavior::IsDone() {
  ROS_DEBUG("[%s]: OrBehavior::IsDone was called", name_->topic.c_str() );
  for( int i = 0; i < children_.size(); i++ )
  {
    if( children_[i]->state.done || children_[i]->state.peer_done ) 
    {
      ROS_DEBUG( "[%s]: state done: %d", name_->topic.c_str(), children_[i]->state.owner.node);
      state_.done = 1;
      return true;
    }
  }

  state_.done = false;
  return false;
  //return state_.done;
}


}
