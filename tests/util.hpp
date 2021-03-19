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
   \file util.hpp
   \author Roland Philippsen
*/

#include "wrap_eigen.hpp"

namespace minitao {
  namespace test {
    
    std::string create_tmpfile(char const * fname_template, char const * contents);
    
    // should also work as-is for minitao::Vector
    bool equal(minitao::Matrix const & lhs, minitao::Matrix const & rhs, double precision);
    bool equal(minitao::Quaternion const & lhs, minitao::Quaternion const & rhs, double precision);
    
    std::string pretty_string(minitao::Vector const & vv);
    std::string pretty_string(minitao::Quaternion const & qq);
    std::string pretty_string(minitao::Matrix const & mm, std::string const & prefix);
    
    void pretty_print(minitao::Vector const & vv, std::ostream & os,
		      std::string const & title, std::string const & prefix, bool nonl = false);
    
    void pretty_print(minitao::Quaternion const & qq, std::ostream & os,
		      std::string const & title, std::string const & prefix, bool nonl = false);
    
    void pretty_print(minitao::Matrix const & mm, std::ostream & os,
		      std::string const & title, std::string const & prefix,
		      bool vecmode = false, bool nonl = false);
    
  }
}
