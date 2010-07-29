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

#ifndef MINITAO_TAO_UTIL_H
#define MINITAO_TAO_UTIL_H

#include <stdexcept>
#include <string>
#include <vector>
#include <map>


class taoNodeRoot;
class taoDNode;


namespace minitao {

  /**
     \note TAO supports multiple joints per link, but all use cases so
     far seem to require that exactly one joint sits between two
     links, so we treat joint names just as link names until further
     notice.
  */
  struct tao_node_info_s {
    tao_node_info_s();
    tao_node_info_s(taoDNode * node, std::string const & link_name,
		    std::string joint_name, double limit_lower, double limit_upper);
    tao_node_info_s(tao_node_info_s const & orig);
    
    int id;
    taoDNode * node;
    std::string link_name;
    std::string joint_name;
    double limit_lower;
    double limit_upper;
  };
  
  
  struct tao_tree_info_s {
    /** deletes the taoNodeRoot. */
    virtual ~tao_tree_info_s();
    taoNodeRoot * root;
    typedef std::vector<tao_node_info_s> node_info_t;
    node_info_t info;
  };

  
  typedef std::map<int, taoDNode *> idToNodeMap_t;
  
  /**
     Create a map between tao nodes and IDs. The \c idToNodeMap is not
     cleared for you: use this function to append to an existing map,
     or clear the map yourself prior to use.
     
     \note Throws a \c runtime_error in case there is a duplicate ID
  */
  void mapNodesToIDs(idToNodeMap_t & idToNodeMap,
		     taoDNode * node)
    throw(std::runtime_error);
  
  
  /**
     Count the total number of links connected to the given node,
     following all children in to the leaf nodes. This number does NOT
     include the given link (because usually you will call this on the
     TAO root node in order to figure out how many degrees of freedom
     the robot has, in which case you do not count the root itself).
  */
  int countNumberOfLinks(taoDNode * root);
  
  
  /**
     Count the total number of joints attached to the given node and
     all its descendants.
  */
  int countNumberOfJoints(taoDNode * node);
  
  
  /**
     Count the total number of degrees of freedom of all the joints
     attached to the given node and all its descendants.
  */
  int countDegreesOfFreedom(taoDNode * node);
  
  
  /**
     Sum up the mass of the given node plus all its descendants.
  */
  double computeTotalMass(taoDNode * node);
  
}

#endif // MINITAO_TAO_UTIL_H
