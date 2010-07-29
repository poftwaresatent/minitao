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
   \file State.cpp
   \author Roland Philippsen
*/

#include "State.hpp"
#include "vector_util.hpp"
#include <string.h>


namespace minitao {
  
  
  State::
  State()
  {
    init(0, 0, 0);
  }
  
  
  State::
  State(State const & orig)
  {
    *this = orig;
  }
  
  
  State::
  State(size_t npos, size_t nvel, size_t nforce)
  {
    init(npos, nvel, nforce);
  }
  
  
  void State::
  init(size_t npos, size_t nvel, size_t nforce)
  {
    time_sec_ = 0;
    time_usec_ = 0;
    position_.resize(npos);
    velocity_.resize(nvel);
    force_.resize(nforce);
    memset(&position_[0], 0, npos * sizeof(double));
    memset(&velocity_[0], 0, nvel * sizeof(double));
    memset(&force_[0], 0, nforce * sizeof(double));
  }
  
  
  State & State::
  operator = (State const & rhs)
  {
    if (&rhs == this) {
      return *this;
    }
    time_sec_ = rhs.time_sec_;
    time_usec_ = rhs.time_usec_;
    position_ = rhs.position_;
    velocity_ = rhs.velocity_;
    force_ = rhs.force_;
    return *this;
  }
  
  
  bool State::
  equal(State const & rhs, int flags, double precision) const
  {
    if (&rhs == this) {
      return true;
    }
    if (flags & COMPARE_ACQUISITION_TIME) {
      if (time_sec_ != rhs.time_sec_) {
	return false;
      }
      if (time_usec_ != rhs.time_usec_) {
	return false;
      }
    }
    if (flags & COMPARE_POSITION) {
      if ( ! compare(position_, rhs.position_, precision)) {
	return false;
      }
    }
    if (flags & COMPARE_VELOCITY) {
      if ( ! compare(velocity_, rhs.velocity_, precision)) {
	return false;
      }
    }
    if (flags & COMPARE_FORCE) {
      if ( ! compare(force_, rhs.force_, precision)) {
	return false;
      }
    }
    return true;
  }
  
}
