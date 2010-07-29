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


#ifndef SFL_STRUTIL_HPP
#define SFL_STRUTIL_HPP


#include <string>
#include <sstream>


namespace minitao {
  
  template<typename displayable_t, typename prefix_t>
  std::string displayString(displayable_t const & displayable, prefix_t const & prefix)
  {
    std::ostringstream result;
    displayable.display(result, prefix);
    return result.str();
  }
  
  /** convert "anything" to a string by using its output operator */
  template<typename Foo>
  std::string to_string(const Foo & foo) {
    std::ostringstream os;
    os << foo;
    return os.str();
  }
  
  /** booleans are better represented by "true" and "false" than by 1 and 0. */
  template<>
  std::string to_string<bool>(const bool & flag);
  
  /** convert a string to "something" based on its input operator */
  template<typename Foo>
  bool string_to(const std::string & str, Foo & foo) {
    Foo bar;
    std::istringstream is(str);
    if( ! (is >> bar))
      return false;
    foo = bar;
    return true;
  }
  
  /** booleans are converted by string matching: "true", "True",
      "TRUE", "on", "On", or "ON" yield a true boolean, whereas
      "false", "False", "FALSE", "off", "Off", or "OFF" yield a false
      boolean. Anything else results in a failure (and foo is not
      touched). */
  template<>
  bool string_to<bool>(const std::string & str, bool & foo);
  
  /** very useful for booleans that are encoded as char, int, short,
      ... sets them to 1 if string_to<bool> yields true, or to 0 if
      string_to<bool> yields false, but doesn't touch foo if
      string_to<bool> failed. */
  template<typename Foo>
  bool string_to_bool(const std::string & str, Foo & foo) {
    bool bar;
    if( ! string_to(str, bar))
      return false;
    foo = bar ? 1 : 0;
    return true;
  }

  
  /**
     Split a string at the first occurrence of a separator, and store
     the two portions in head and tail. For example,
     "head:tail_or:whatever::comes:then" with separator ':' will yield
     "head" and "tail_or:whatever::comes:then". The same string split
     along '_' would yield "head:tail" and "or:whatever::comes:then".
     
     Note that it's OK to pass the same instance as input and tail,
     but DO NOT pass the head twice.
     
     \code
     for (int ii(1); ii < argc; ++ii) {
       string head;
       string tail(argv[ii]);
       while (splitstring(tail, ':', head, tail))
         cout << head << "\n";
       cout << head << "\n";
     }
     \endcode
     
     \return true if there is more to be extracted, allowing you to
     easily tokenize a string. But see also tokenize() which does just
     that.
  */
  bool splitstring(std::string const & input, char separator,
		   std::string & head, std::string & tail);
  
  
  /**
     For any tokenlist_t that accepts push_back(string const &) and
     can return its size().
  */
  template<typename tokenlist_t>
  size_t tokenize(std::string const & input, char separator, tokenlist_t & output) {
    std::string head;
    std::string tail(input);
    while (splitstring(tail, separator, head, tail))
      output.push_back(head);
    output.push_back(head);
    return output.size();
  }
  
  
  /**
     For any tokenlist_t whose operator[]() returns a string const &
     and which can return its size().
  */
  template<typename tokenlist_t, typename value_t>
  bool token_to(tokenlist_t const & tokenlist, size_t index, value_t & value) {
    if (index >= tokenlist.size())
      return false;
    return string_to(tokenlist[index], value);
  }
  
}

#endif // SFL_STRUTIL_HPP
