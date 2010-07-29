/*
 * MiniTAO http://gitorious.org/minitao
 *
 * Copyright (c) 2010 Stanford University. All rights reserved.
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
   \file vector_util.hpp
   \author Roland Philippsen
*/

#ifndef MINITAO_VECTOR_UTIL_HPP
#define MINITAO_VECTOR_UTIL_HPP

#include <vector>
#include <iosfwd>
#include <math.h>


namespace minitao {
  
  
  template<typename container_t>
  bool compare(container_t const & lhs, container_t const & rhs, typename container_t::value_type precision)
  {
    if (&lhs == &rhs) {
      return true;
    }
    if (lhs.size() != rhs.size()) {
      return false;
    }
    typename container_t::const_iterator il(lhs.begin());
    typename container_t::const_iterator ir(rhs.begin());
    typename container_t::const_iterator il_end(lhs.end());
    for (/**/; il != il_end; ++il, ++ir) {
      if (fabs(*il - *ir) > precision) {
	return false;
      }
    }
    return true;
  }
  
  
  template<typename container_t>
  void zero(container_t & vv)
  {
    std::fill(vv.begin(), vv.end(), 0);
  }
  
}

namespace std {
  
  ostream & operator << (ostream & os, vector<double> const & rhs);
  
}

#endif // MINITAO_VECTOR_UTIL_HPP
