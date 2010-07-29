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
   \file vector_util.cpp
   \author Roland Philippsen
*/

#include "vector_util.hpp"
#include <iostream>
#include <string.h>
#include <stdio.h>


namespace std {
  
  ostream & operator << (ostream & os, vector<double> const & rhs)
  {
    if ( ! rhs.empty()) {    
      static int const buflen(32);
      char buf[buflen];
      memset(buf, 0, sizeof(buf));
      for (vector<double>::const_iterator ii(rhs.begin()); ii != rhs.end(); ++ii) {

#ifndef WIN32

	if (isinf(*ii)) {
	  snprintf(buf, buflen-1, " inf    ");
	}
	else if (isnan(*ii)) {
	  snprintf(buf, buflen-1, " nan    ");
	}
	else if (fabs(fmod(*ii, 1)) < 1e-6) {
	  snprintf(buf, buflen-1, "%- 7d  ", static_cast<int>(rint(*ii)));
	}
	else {
	  snprintf(buf, buflen-1, "% 6.4f  ", *ii);
	}

#else

	sprintf_s(buf, buflen-1, "% 6.4f  ", *ii);

#endif // WIN32

	os << buf;
      }
    }
    return os;
  }
  
}
