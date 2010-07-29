/*
 * MiniTAO http://gitorious.org/minitao
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
   \file wrap_eigen.hpp
   \author Roland Philippsen
*/

#ifndef MINITAO_WRAP_EIGEN_HPP
#define MINITAO_WRAP_EIGEN_HPP

#include <Eigen/Geometry>

namespace minitao {
  typedef Eigen::Transform3d Transform;
  typedef Eigen::Translation3d Translation;
  typedef Eigen::Quaternion<double> Quaternion;
  typedef Eigen::VectorXd Vector;
  typedef Eigen::MatrixXd Matrix;
}

#endif // MINITAO_WRAP_EIGEN_HPP
