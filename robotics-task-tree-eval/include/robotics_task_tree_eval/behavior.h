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
#ifndef INCLUDE_BEHAVIOR_H_
#define INCLUDE_BEHAVIOR_H_

#include <queue>
#include "robotics_task_tree_eval/node.h"
#include "remote_mutex/remote_mutex.h"

enum ROBOT {
  PR2=0, 
  BAXTER=1
} ;

#define BEHAVIOR_SLEEP_TIME 10

namespace task_net {
class Behavior: public Node {
 public:
  Behavior();
  Behavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(BEHAVIOR_SLEEP_TIME));
  virtual ~Behavior();

 private:
};

class ThenBehavior: public Behavior {
 public:
  ThenBehavior();
  ThenBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(BEHAVIOR_SLEEP_TIME));
  virtual ~ThenBehavior();
  void UpdateActivationPotential();
 protected:
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
 private:
  std::queue<NodeId_t*> activation_queue_;
};
class AndBehavior: public Behavior {
 public:
  AndBehavior();
  AndBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(BEHAVIOR_SLEEP_TIME));
  virtual ~AndBehavior();
  void UpdateActivationPotential();
 protected:
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
};
class OrBehavior: public Behavior {
 public:
  OrBehavior();
  OrBehavior(NodeId_t name, NodeList peers, NodeList children,
    NodeId_t parent,
    State_t state,
    bool use_local_callback_queue = false,
    boost::posix_time::millisec mtime = boost::posix_time::millisec(BEHAVIOR_SLEEP_TIME));
  virtual ~OrBehavior();
  void UpdateActivationPotential();
 protected:
  virtual bool Precondition();
  virtual uint32_t SpreadActivation();
 private:
  uint32_t seed;
  uint32_t random_child_selection;
};

class WhileBehavior: public Behavior {};
}  // namespace task_net
#endif  // INCLUDE_BEHAVIOR_H_
