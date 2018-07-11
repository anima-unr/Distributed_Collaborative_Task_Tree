/*
dummy_behavior
Copyright (C) 2017  Janelle Blankenburg, David Feil-Seifer

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
#ifndef DUMMY_BEHAVIOR_H_PICK
#define DUMMY_BEHAVIOR_H_PICK

#include "robotics_task_tree_eval/behavior.h"
#include "remote_mutex/remote_mutex.h"
#include <table_task_sim/SimState.h>
#include <table_task_sim/header.h>


namespace task_net {
	class DummyBehaviorPick: public Behavior {
	 public:
	  DummyBehaviorPick();
	  DummyBehaviorPick(NodeId_t name, NodeList peers, NodeList children,
	    NodeId_t parent,
	    State_t state,
	    std::string object,
	    ROBOT robot_des,
	    bool init_v,
	    bool use_local_callback_queue = false,
	    boost::posix_time::millisec mtime = boost::posix_time::millisec(BEHAVIOR_SLEEP_TIME*1.5));
	  virtual ~DummyBehaviorPick();
	  
	  void UpdateActivationPotential();
	  bool ActivationPrecondition();
	  void Pick(std::string object, ROBOT robot_des);
	  bool PickDone();
	  void Work();
	  void StateCallback( table_task_sim::SimState msg);
	  
	 protected:
	  virtual bool Precondition();
	  virtual uint32_t SpreadActivation();
	  mutex::RemoteMutex mut_arm;
	  std::string object_;
	  ROBOT robot_des_;


	  ros::Subscriber state_sub_;
	  table_task_sim::SimState table_state_;
	  bool init_v_;
	  
	};

}

#endif
