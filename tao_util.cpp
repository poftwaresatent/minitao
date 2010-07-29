/*
 * MiniTAO http://gitorious.org/minitao
 *
 * Copyright (c) 1997-2009 Stanford University. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tao_util.hpp"
#include "strutil.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDNode.h>
#include <tao/dynamics/taoJoint.h>


namespace minitao {
  
  void mapNodesToIDs(idToNodeMap_t & idToNodeMap,
		     taoDNode * node)
    throw(std::runtime_error)
  {
    deInt id = node->getID();
    if (idToNodeMap.find( id ) != idToNodeMap.end())
      throw std::runtime_error("minitao::mapNodesToIDs(): duplicate ID " + to_string(id));
    idToNodeMap.insert(std::make_pair(id, node));
    
    // recurse
    for( taoDNode* p = node->getDChild(); p != NULL; p = p->getDSibling() )
      mapNodesToIDs(idToNodeMap, p);
  }
  
  
  int countNumberOfLinks(taoDNode * root)
  {
    int count(0);
    for (taoDNode * child(root->getDChild()); child != NULL; child = child->getDSibling()) {
      ++count;
      count += countNumberOfLinks(child);
    }
    return count;
  }
  
  
  int countNumberOfJoints(taoDNode * node)
  {
    int count(0);
    for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
      ++count;
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      count += countNumberOfJoints(child);
    }
    return count;
  }
  
  
  int countDegreesOfFreedom(taoDNode * node)
  {
    int dof(0);
    for (taoJoint * joint(node->getJointList()); 0 != joint; joint = joint->getNext()) {
      dof += joint->getDOF();
    }
    for (taoDNode * child(node->getDChild()); 0 != child; child = child->getDSibling()) {
      dof += countDegreesOfFreedom(child);
    }
    return dof;
  }
  
  
  double computeTotalMass(taoDNode * node)
  {
    double mass(0);
    if (node->mass()) {
      // I guess TAO nodes always have a mass, but the interface
      // returns a pointer, so maybe there are cases where there is
      // not even a zero mass? Whatever, just be paranoid and check
      // for non-NULL pointers.
      mass = *node->mass();
    }
    for (taoDNode * child(node->getDChild()); child != NULL; child = child->getDSibling()) {
      mass += computeTotalMass(child);
    }
    return mass;
  }
  
  
  tao_node_info_s::
  tao_node_info_s()
    : id(-2),
      node(0),
      link_name(""),
      joint_name(""),
      limit_lower(0),
      limit_upper(0)
  {
  }
  
  
  tao_node_info_s::
  tao_node_info_s(taoDNode * _node,
		  std::string const & _link_name,
		  std::string _joint_name,
		  double _limit_lower,
		  double _limit_upper)
    : id(_node->getID()),
      node(_node),
      link_name(_link_name),
      joint_name(_joint_name),
      limit_lower(_limit_lower),
      limit_upper(_limit_upper)
  {
  }
  
  
  tao_node_info_s::
  tao_node_info_s(tao_node_info_s const & orig)
    : id(orig.id),
      node(orig.node),
      link_name(orig.link_name),
      joint_name(orig.joint_name),
      limit_lower(orig.limit_lower),
      limit_upper(orig.limit_upper)
  {
  }
  
  
  tao_tree_info_s::
  ~tao_tree_info_s()
  {
    delete root;
  }
  
}
