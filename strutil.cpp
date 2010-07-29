/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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

#include "strutil.hpp"
using namespace std;

namespace minitao {
  
  template<>
  std::string to_string<bool>(const bool & flag) {
    if (flag)
      return "true";
    return "false";      
  }
  
  
  template<>
  bool string_to<bool>(const std::string & str, bool & foo) {
    if((str == "true") || (str == "TRUE") || (str == "True")
       || (str == "on") || (str == "ON") || (str == "On")){
      foo = true;
      return true;
    }
    else if((str == "false") || (str == "FALSE") || (str == "False")
	    || (str == "off") || (str == "OFF") || (str == "Off")){
      foo = false;
      return true;
    }
    return false;
  }
  
  
  bool splitstring(string const & input, char separator,
		   string & head, string & tail)
  {
    if (input.empty()) {
      head = "";
      tail = "";
      return false;
    }
    string::size_type col(input.find(separator, 0));
    head = input.substr(0, col);
    if (string::npos != col)
      tail = input.substr(col + 1);
    else
      tail = "";
    return true;
  }
  
}
