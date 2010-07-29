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
   \file State.hpp
   \author Roland Philippsen
*/

#ifndef MINITAO_STATE_HPP
#define MINITAO_STATE_HPP

#include <vector>

namespace minitao {
  
  class State
  {
  public:
    typedef enum {
      COMPARE_ACQUISITION_TIME = 0x1,
      COMPARE_POSITION         = 0x2,
      COMPARE_VELOCITY         = 0x4,
      COMPARE_FORCE            = 0x8,
      COMPARE_ALL              = 0xf
    } compare_flags_t;
    
    State();
    State(State const & orig);
    State(size_t npos, size_t nvel, size_t nforce);
    
    void init(size_t npos, size_t nvel, size_t nforce);
    
    bool equal(State const & rhs,
	       int flags = COMPARE_POSITION | COMPARE_VELOCITY,
	       double precision = 1e-3) const;
    
    State & operator = (State const & rhs);
    
    size_t time_sec_;
    size_t time_usec_;
    std::vector<double> position_;
    std::vector<double> velocity_;
    std::vector<double> force_;
  };
  
}

#endif // MINITAO_STATE_HPP
