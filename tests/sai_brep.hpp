/*
 * Stanford Whole-Body Control Framework http://stanford-wbc.sourceforge.net/
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program.  If not, see
 * <http://www.gnu.org/licenses/>
 */

/**
   \file sai_brep.hpp
   \author Luis Sentis (copy-paste-adapted by Roland Philippsen)
*/

#ifndef JSPACE_TESTS_SAI_BREP_HPP
#define JSPACE_TESTS_SAI_BREP_HPP

#include "tao_util.hpp"
#include "wrap_eigen.hpp"
#include <tao/matrix/TaoDeMath.h>	
#include <map>
#include <string>
#include <vector>

class taoDNode;

namespace minitao {
  
  namespace test {
    
    class BranchingRepresentation {
      friend class BRParser;
      BranchingRepresentation();
      
    public:
      ~BranchingRepresentation();
      
      inline taoDNode * rootNode() { return rootNode_; }
    
    private:
      taoDNode * rootNode_; 
      idToNodeMap_t idToNodeMap_; 
      std::map<std::string, taoDNode*> linkNameToNodeMap_;
      std::map<std::string, taoDNode*> jointNameToNodeMap_;
      mutable std::map<std::string, taoDNode*> linkNameToNodeMapWithAliases_;
      mutable std::map<std::string, taoDNode*> jointNameToNodeMapWithAliases_;
      int numJoints_;
      double totalMass_;
      minitao::Vector grav_;

      minitao::Vector defaultJointPosVec_;
      minitao::Vector upperJointLimitVec_; //hard jointLimits
      minitao::Vector lowerJointLimitVec_; //hard jointLimits
      
      taoDNode* findNodeID( taoDNode*, int);
    };
    
  }
}

#endif // JSPACE_TESTS_SAI_BREP_HPP
