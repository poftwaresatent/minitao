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

/**
   \file tao_dump.hpp
   \author Roland Philippsen
*/

#ifndef MINITAO_UTIL_DUMP_HPP
#define MINITAO_UTIL_DUMP_HPP

#include <iosfwd>
#include <vector>
#include <stdexcept>

class deVector6;
class deVector3;
class deQuaternion;
class deFrame;
class taoJoint;
class deMassProp;
class deMatrix3;
class taoDNode;


namespace minitao {
  
  struct tao_tree_info_s;
  
  std::string inertia_matrix_to_string(deMatrix3 const & mx);
  
  /**
     Write a textual description of the TAO tree rooted at the given
     node to the given std::ostream. Each line of output is prefixed
     by the given \c prefix, and each level of the tree is indented
     two spaces with respect to the previous level. If the \c detailed
     flag is specified, then a lot more information is given for each
     node. Optionally, you can pass pointers to vectors containing the
     node and joint names, which will then be output along with the ID
     (if there is an entry for that ID in the given \c
     id_to_link_name). Use \c NULL for \c id_to_link_name and/or \c
     id_to_link_name if you do not have that information, or don't
     care to have it printed.
  */
  void dump_tao_tree(std::ostream & os, taoDNode * root, std::string prefix,
		     bool detailed,
		     std::vector<std::string> * id_to_link_name,
		     std::vector<std::string> * id_to_joint_name);
  
  /**
     Similar to dump_tao_tree() but uses the more recent tao_tree_info_s structure.
  */
  void dump_tao_tree_info(std::ostream & os, tao_tree_info_s * tree, std::string prefix, bool detailed);
  
  /**
     Similar to dump_tao_tree_info(), but attempts to spew it out in a
     format that can be read back into a parser.
  */
  void xmldump_tao_tree_info(std::ostream & os, tao_tree_info_s * tree) throw(std::runtime_error);
  
}

namespace std {
  
  ostream & operator << (ostream & os, deVector6 const & vec);
  ostream & operator << (ostream & os, deVector3 const & vec);
  ostream & operator << (ostream & os, deQuaternion const & vec);
  ostream & operator << (ostream & os, deFrame const & frame);
  ostream & operator << (ostream & os, taoJoint /*const*/ & joint);
  ostream & operator << (ostream & os, deMassProp const & rhs);
  
}

#endif // MINITAO_UTIL_DUMP_HPP
