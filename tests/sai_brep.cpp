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
   \file sai_brep.cpp
   \author Luis Sentis (copy-paste-adapted by Roland Philippsen)
*/

#include "sai_brep.hpp"
#include <tao/dynamics/taoNode.h>

using namespace std;

namespace minitao {
  namespace test {
    
    
    BranchingRepresentation::
    BranchingRepresentation()
      : numJoints_(0), totalMass_(0.0), grav_(3)
    {
    }


    BranchingRepresentation::
    ~BranchingRepresentation()
    {
      // Do NOT delete rootNode_ because others might still be using it.
    }


    taoDNode* 
    BranchingRepresentation::findNodeID( taoDNode* node, int id ) {

      deInt tmpID = node->getID();
      if( tmpID == id ) return node;
      taoDNode* foundNode = NULL;	
      for( taoDNode* n = node->getDChild(); n!= NULL && foundNode == NULL; n = node->getDSibling() )
	foundNode = findNodeID( n, id );
      return foundNode;
    }
    
  }
}
