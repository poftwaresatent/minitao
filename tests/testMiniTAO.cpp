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
   \file testJspace.cpp
   \author Roland Philippsen
*/

#include "model_library.hpp"
#include "util.hpp"
#include "sai_brep_parser.hpp"
#include "strutil.hpp"
#include "tao_dump.hpp"
#include "vector_util.hpp"
#include <tao/dynamics/taoNode.h>
#include <tao/dynamics/taoDynamics.h>
#include <tao/dynamics/taoJoint.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <gtest/gtest.h>
#include <errno.h>

#include <eigen2/Eigen/SVD>
#include <eigen2/Eigen/LU>

using namespace std;
using namespace minitao::test;

static std::string create_puma_frames() throw(runtime_error);


TEST (jspaceModel, state)
{
  minitao::Model * model(0);
  try {
    model = create_puma_model();
    int const ndof(model->getNDOF());
    minitao::State state_in(ndof, ndof, 0);
    for (int ii(0); ii < ndof; ++ii) {
      state_in.position_[ii] = -3 + ii * 0.5;
      state_in.velocity_[ii] = 3 - ii * 0.25;
    }
    model->setState(state_in);
    {
      minitao::State state_out(ndof, ndof, 0);
      state_out = model->getState();
      EXPECT_TRUE (state_out.equal(state_in, minitao::State::COMPARE_ALL, 1e-6))
	<< "minitao::State assignment or minitao::Model::getState() is buggy";
    }
    {
      minitao::State state_out(model->getState());
      EXPECT_TRUE (state_out.equal(state_in, minitao::State::COMPARE_ALL, 1e-6))
	<< "minitao::State copy ctor or minitao::Model::getState() is buggy";
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, branching)
{
  minitao::Model * model(0);
  try {
    model = create_puma_model();
    EXPECT_EQ (model->getNNodes(), 6) << "Puma should have 6 nodes";
    EXPECT_EQ (model->getNJoints(), 6) << "Puma should have 6 joints";
    EXPECT_EQ (model->getNDOF(), 6) << "Puma should have 6 DOF";
    char const * node_name[] = {
      "base",
      "upper_arm",
      "lower_arm",
      "wrist-hand",
      "wrist-finger",
      "end-effector"
    };
    char const * joint_name[] = {
      "shoulder-yaw",
      "shoulder-pitch",
      "elbow",
      "wrist-roll1",
      "wrist-pitch",
      "wrist-roll2"
    };
    for (int ii(0); ii < 6; ++ii) {
      EXPECT_NE ((void*)0, model->getNode(ii))
	<< "Could not get node " << ii;
      EXPECT_NE ((void*)0, model->getNodeByName(node_name[ii]))
	<< "Could not get node by name \"" << node_name[ii] << "\"";
      if (model->getNodeByName(node_name[ii])) {
	EXPECT_EQ (ii, model->getNodeByName(node_name[ii])->getID())
	  << "Node with name \"" << node_name[ii] << "\" should have ID " << ii;
      }
      EXPECT_NE ((void*)0, model->getNodeByJointName(joint_name[ii]))
	<< "Could not get node by joint name \"" << joint_name[ii] << "\"";
      if (model->getNodeByJointName(joint_name[ii])) {
	EXPECT_EQ (ii, model->getNodeByJointName(joint_name[ii])->getID())
	  << "Node with joint name \"" << joint_name[ii] << "\" should have ID " << ii;
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, kinematics)
{
  minitao::Model * model(0);
  try {
    model = create_puma_model();
    int const ndof(model->getNDOF());
    minitao::State state(ndof, ndof, 0);
    
    string const frames_filename(create_puma_frames());
    ifstream is(frames_filename.c_str());

    string line;
    int joint_positions_count(0);
    int link_origins_count(0);
    int line_count(0);

    while (getline(is, line)) {
      ++line_count;

      if (line.empty() || ('=' == line[0])) {
	continue;
      }
      vector<string> token;
      if (1 > minitao::tokenize(line, ':', token)) {
	continue;
      }
      
      if ("joint_positions" == token[0]) {
	if (2 > token.size()) {
	  FAIL () << frames_filename << ": line " << line_count << ": no joint positions, expected " << ndof;
	}
	istringstream ipos(token[1]);
	for (int ii(0); ii < ndof; ++ii) {
	  ipos >> state.position_[ii];
	}
	if ( ! ipos) {
	  FAIL () << frames_filename << ": line " << line_count << ": not enough joint positions, expected " << ndof;
	}
	model->update(state);
	++joint_positions_count;
	continue;
      }
      
      if ("link_origins" == token[0]) {
	++link_origins_count;
	continue;
      }
      
      if (4 > token.size()) {
	FAIL () << frames_filename << ": line " << line_count << ": expected ID-frame entry";
      }
      
      if (joint_positions_count != link_origins_count) {
	FAIL () << frames_filename << ": line " << line_count << ": joint_positions_count != link_origins_count";
      }
      
      int id;
      float rx, ry, rz, rw, tx, ty, tz;
      if (8 != sscanf(line.c_str(),
		      " ID %d: r: { %f %f %f %f } t: { %f %f %f }",
		      &id, &rx, &ry, &rz, &rw, &tx, &ty, &tz)) {
	FAIL () << frames_filename << ": line " << line_count << ": could not parse ID-frame entry";
      }
      if (ndof <= id) {
	FAIL () << frames_filename << ": line " << line_count << ": ID-frame entry " << id << " exceeds NDOF " << ndof;
      }
      
      minitao::Transform transform;
      if ( ! model->getGlobalFrame(model->getNode(id), transform)) {
	FAIL() << frames_filename << ": line " << line_count << ": could not get global frame " << id << " from model";
      }
      minitao::Quaternion quat_computed(transform.rotation());
      minitao::Quaternion quat_expected(rw, rx, ry, rz);
      double const delta_angle(quat_computed.angularDistance(quat_expected));
      EXPECT_TRUE (fabs(delta_angle) < 0.15 * M_PI / 180.0) // looks like it's hard to be more precise...
	<< "rotation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << pretty_string(quat_expected) << "\n"
	<< "  computed: " << pretty_string(quat_computed) << "\n"
	<< "  delta_angle: " << delta_angle * 180 / M_PI << " deg\n";
      Eigen::Vector3d trans_expected(tx, ty, tz);
      EXPECT_TRUE (equal(transform.translation(), trans_expected, 1e-6))
	<< "translation mismatch\n"
	<< "  entry: " << joint_positions_count << "\n"
	<< "  pos: " << state.position_ << "\n"
	<< "  ID: " << id << "\n"
	<< "  expected: " << pretty_string(trans_expected) << "\n"
	<< "  computed: " << pretty_string(transform.translation());
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


static double smart_delta(double have, double want)
{
  if (fabs(want) > 1e-6) {
    return (have - want) / want;
  }
  return have - want;
}


template<typename mtype>
int mnrows(mtype const & mm) { return mm.rows(); }

template<>
int mnrows(deMatrix3 const & mm) { return 3; }

template<typename mtype>
int mncols(mtype const & mm) { return mm.cols(); }

template<>
int mncols(deMatrix3 const & mm) { return 3; }


template<typename mtype>
bool check_matrix(char const * name,
		  mtype const & want,
		  mtype const & have,
		  double precision,
		  std::ostringstream & msg)
{
  int const nrows(mnrows(want));
  if (nrows != mnrows(have)) {
    msg << "check_matrix(" << name << ") size mismatch: have " << mnrows(have)
	<< " rows but want " << nrows << "\n";
    return false;
  }
  int const ncolumns(mncols(want));
  if (ncolumns != mncols(have)) {
    msg << "check_matrix(" << name << ") size mismatch: have " << mncols(have)
	<< " columns but want " << ncolumns << "\n";
    return false;
  }
  
  precision = fabs(precision);
  double maxdelta(0);
  minitao::Matrix delta(nrows, ncolumns);
  for (int ii(0); ii < nrows; ++ii) {
    for (int jj(0); jj < ncolumns; ++jj) {
      delta.coeffRef(ii, jj) = fabs(smart_delta(have.coeff(ii, jj), want.coeff(ii, jj)));
      if (delta.coeff(ii, jj) > precision) {
	maxdelta = delta.coeff(ii, jj);
      }
    }
  }
  double const halfmax(0.5 * maxdelta);
  double const tenprecision(10 * precision);
  
  if (maxdelta <= precision) {
    msg << "check_matrix(" << name << ") OK\n";
  }
  else {
    msg << "check_matrix(" << name << ") FAILED\n";
  }
  msg << "  precision = " << precision << "\n"
      << "  maxdelta = " << maxdelta << "\n";
  pretty_print(delta, msg, "  delta", "    ");
  msg << "  error pattern\n";
  for (int ii(0); ii < nrows; ++ii) {
    msg << "    ";
    for (int jj(0); jj < ncolumns; ++jj) {
      if (delta.coeff(ii, jj) <= precision) {
	if (delta.coeff(ii, jj) < halfmax) {
	  msg << ".";
	}
	else {
	  msg << "o";
	}
      }
      else if (delta.coeff(ii, jj) >= tenprecision) {
	msg << "#";
      }
      else {
	msg << "*";
      }
    }
    msg << "\n";
  }
  
  return maxdelta <= precision;
}


template<typename vtype>
void print_vector(vtype const & vv,
		  std::ostream & msg,
		  std::string const & title,
		  std::string const & prefix)
{
  msg << title << "\n" << prefix << vv << "\n";
}


template<>
void print_vector(minitao::Vector const & vv,
		  std::ostream & msg,
		  std::string const & title,
		  std::string const & prefix)
{
  pretty_print(vv, msg, title, prefix);
}


template<typename vtype>
int vsize(vtype const & vv) { return vv.size(); }

template<>
int vsize(deVector3 const & vv) { return 3; }

template<>
int vsize(deVector6 const & vv) { return 6; }


template<typename vtype>
double vget(vtype const & vv, int idx) { return vv[idx]; }

template<>
double vget(deVector6 const & vv, int idx) { return vv.elementAt(idx); }


template<typename vtype>
bool check_vector(char const * name,
		  vtype const & want,
		  vtype const & have,
		  double precision,
		  std::ostream & msg)
{
  int const nelems(vsize(want));
  if (nelems != vsize(have)) {
    msg << "check_vector(" << name << ") size mismatch: have " << vsize(have)
	<< " elements but want " << nelems << "\n";
    return false;
  }
  
  precision = fabs(precision);
  double maxdelta(0);
  minitao::Vector delta(nelems);
  for (int ii(0); ii < nelems; ++ii) {
    delta.coeffRef(ii) = fabs(smart_delta(vget(have, ii), vget(want, ii)));
    if (delta.coeff(ii) > precision) {
      maxdelta = delta.coeff(ii);
    }
  }
  double const halfmax(0.5 * maxdelta);
  double const tenprecision(10 * precision);
  
  if (maxdelta <= precision) {
    msg << "check_vector(" << name << ") OK\n";
  }
  else {
    msg << "check_vector(" << name << ") FAILED\n";
  }
  msg << "  precision = " << precision << "\n"
      << "  maxdelta = " << maxdelta << "\n";
  print_vector(delta, msg, "  delta", "    ");
  msg << "  error pattern\n    ";
  for (int ii(0); ii < nelems; ++ii) {
    if (delta.coeff(ii) <= precision) {
      if (delta.coeff(ii) < halfmax) {
	msg << ".";
      }
      else {
	msg << "o";
      }
    }
    else if (delta.coeff(ii) >= tenprecision) {
      msg << "#";
    }
    else {
      msg << "*";
    }
  }
  msg << "\n";
  
  return maxdelta <= precision;
}


TEST (jspaceModel, Jacobian_R)
{
  minitao::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    taoDNode * ee(model->getNode(0));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 0)";
    minitao::State state(2, 2, 0); // here we're only gonna test the first joint though
    minitao::zero(state.position_);
    minitao::zero(state.velocity_);
    minitao::Transform ee_lframe(Eigen::Translation<double, 3>(0, 1, 0));
    
    for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
      state.position_[0] = qq;
      model->update(state);
      
      double const q1(state.position_[0]);
      double const c1(cos(q1));
      double const s1(sin(q1));
      
      minitao::Transform ee_gframe;
      ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
      minitao::Vector const ee_gpos(ee_gframe.translation());
      {
	minitao::Vector ee_gpos_check(3);
	ee_gpos_check.coeffRef(1) = c1;
	ee_gpos_check.coeffRef(2) = s1;
	std::ostringstream msg;
	msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n"
	    << "  want: " << pretty_string(ee_gpos_check) << "\n"
	    << "  have: " << pretty_string(ee_gpos) << "\n";
	bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	EXPECT_TRUE (gpos_ok) << msg.str();
	if ( ! gpos_ok) {
	  continue;		// no use checking Jg as well, it'll be off
	}
      }
      
      minitao::Matrix Jg_all(6, 2);
      ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg_all));
      minitao::Matrix const Jg(Jg_all.block(0, 0, 6, 1));
      {
	minitao::Matrix Jg_check(minitao::Matrix::Zero(6, 1));
	Jg_check.coeffRef(1, 0) = -s1;
	Jg_check.coeffRef(2, 0) =  c1;
	Jg_check.coeffRef(3, 0) =  1;
	std::ostringstream msg;
	msg << "Checking Jacobian for q = " << state.position_ << "\n";
	pretty_print(Jg_check, msg, "  want", "    ");
	pretty_print(Jg, msg, "  have", "    ");
	EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, Jacobian_RR)
{
  minitao::Model * model(0);
  try {
    model = create_unit_mass_RR_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    minitao::State state(2, 2, 0);
    minitao::zero(state.velocity_);
    minitao::Transform ee_lframe(Eigen::Translation<double, 3>(0, 1, 0));
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const c12(cos(q1+q2));
	double const s1(sin(q1));
	double const s12(sin(q1+q2));
	
	minitao::Transform ee_gframe;
	ASSERT_TRUE (model->computeGlobalFrame(ee, ee_lframe, ee_gframe));
	minitao::Vector const ee_gpos(ee_gframe.translation());
	{
	  minitao::Vector ee_gpos_check(3);
	  ee_gpos_check.coeffRef(1) = c1 + c12;
	  ee_gpos_check.coeffRef(2) = s1 + s12;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  pretty_print(ee_gpos_check, msg, "  want", "    ");
	  pretty_print(ee_gpos, msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	minitao::Matrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, ee_gpos, Jg));
	{
	  minitao::Matrix Jg_check(minitao::Matrix::Zero(6, 2));
	  Jg_check.coeffRef(1, 0) = -s1 - s12;
	  Jg_check.coeffRef(2, 0) =  c1 + c12;
	  Jg_check.coeffRef(3, 0) =  1;
	  Jg_check.coeffRef(1, 1) = -s12;
	  Jg_check.coeffRef(2, 1) =  c12;
	  Jg_check.coeffRef(3, 1) =  1;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  pretty_print(Jg_check, msg, "  want", "    ");
	  pretty_print(Jg, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, Jacobian_RP)
{
  minitao::Model * model(0);
  try {
    model = create_unit_mass_RP_model();
    taoDNode * ee(model->getNode(1));
    ASSERT_NE ((void*)0, ee) << "no end effector (node ID 1)";
    minitao::State state(2, 2, 0);
    minitao::zero(state.velocity_);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	{
	  minitao::Transform ee_gframe;
	  ASSERT_TRUE (model->getGlobalFrame(ee, ee_gframe));
	  minitao::Vector const ee_gpos(ee_gframe.translation());
	  minitao::Vector ee_gpos_check(3);
	  ee_gpos_check.coeffRef(1) = c1 - s1 * q2;
	  ee_gpos_check.coeffRef(2) = s1 + c1 * q2;
	  std::ostringstream msg;
	  msg << "Verifying end-effector frame (position only) for q = " << state.position_ << "\n";
	  pretty_print(ee_gpos_check, msg, "  want", "    ");
	  pretty_print(ee_gpos, msg, "  have", "    ");
	  bool const gpos_ok(check_vector("ee_pos", ee_gpos_check, ee_gpos, 1e-3, msg));
	  EXPECT_TRUE (gpos_ok) << msg.str();
	  if ( ! gpos_ok) {
	    continue;		// no use checking Jg as well, it'll be off
	  }
	}
	
	minitao::Matrix Jg(6, 2);
	ASSERT_TRUE (model->computeJacobian(ee, Jg));
	{
	  minitao::Matrix Jg_check(minitao::Matrix::Zero(6, 2));
	  Jg_check.coeffRef(1, 0) = -s1 - c1 * q2;
	  Jg_check.coeffRef(2, 0) =  c1 - s1 * q2;
	  Jg_check.coeffRef(3, 0) =  1;
	  Jg_check.coeffRef(1, 1) = -s1;
	  Jg_check.coeffRef(2, 1) =  c1;
	  Jg_check.coeffRef(3, 1) =  0;
	  std::ostringstream msg;
	  msg << "Checking Jacobian for q = " << state.position_ << "\n";
	  pretty_print(Jg_check, msg, "  want", "    ");
	  pretty_print(Jg, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("Jacobian", Jg_check, Jg, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


static void compute_unit_mass_RR_mass_inertia(double q1, double q2, minitao::Matrix & MM_check)
{
  double const c1(cos(q1));
  double const c12(cos(q1+q2));
  double const s1(sin(q1));
  double const s12(sin(q1+q2));
  MM_check.resize(2, 2);
  MM_check.coeffRef(0, 0) = 1 + pow(s1 + s12, 2) + pow(c1 + c12, 2);
  MM_check.coeffRef(1, 0) = s12 * (s1 + s12) + c12 * (c1 + c12);
  MM_check.coeffRef(0, 1) = MM_check.coeff(1, 0);
  MM_check.coeffRef(1, 1) = 1;
}


static void compute_unit_inertia_RR_mass_inertia(double q1, double q2, minitao::Matrix & MM_check)
{
  double const c1(cos(q1));
  double const c12(cos(q1+q2));
  double const s1(sin(q1));
  double const s12(sin(q1+q2));
  MM_check.resize(2, 2);
  MM_check.coeffRef(0, 0) = 3 + pow(2*s1 + s12, 2) + pow(2*c1 + c12, 2);
  MM_check.coeffRef(1, 0) = 1 + s12 * (2*s1 + s12) + c12 * (2*c1 + c12);
  MM_check.coeffRef(0, 1) = MM_check.coeff(1, 0);
  MM_check.coeffRef(1, 1) = 2;
}


TEST (jspaceModel, mass_inertia_RR)
{
  typedef minitao::Model * (*create_model_t)();
  create_model_t create_model[] = {
    create_unit_mass_RR_model,
    create_unit_inertia_RR_model
  };
  
  typedef void (*compute_mass_inertia_t)(double, double, minitao::Matrix &);
  compute_mass_inertia_t compute_mass_inertia[] = {
    compute_unit_mass_RR_mass_inertia,
    compute_unit_inertia_RR_mass_inertia
  };
  
  for (size_t test_index(0); test_index < 2; ++test_index) {
    minitao::Model * model(0);
    try {
      model = create_model[test_index]();
      taoDNode * n1(model->getNode(0));
      taoDNode * n2(model->getNode(1));
      ASSERT_NE ((void*)0, n1);
      ASSERT_NE ((void*)0, n2);
      minitao::State state(2, 2, 0);
      minitao::zero(state.velocity_);
      
      for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
	for (double q2(-M_PI); q2 <= M_PI; q2 += 2 * M_PI / 7) {
	  state.position_[0] = q1;
	  state.position_[1] = q2;
	  model->update(state);
	  
	  minitao::Matrix MM(2, 2);
	  model->getMassInertia(MM);
	  minitao::Matrix MM_check;
	  compute_mass_inertia[test_index](q1, q2, MM_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    pretty_print(MM_check, msg, "  want", "    ");
	    pretty_print(MM, msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	  }
	  
	  minitao::Matrix MMinv(2, 2);
	  model->getInverseMassInertia(MMinv);
	  minitao::Matrix MMinv_check(2, 2);
	  MM_check.computeInverse(&MMinv_check);
	  {
	    std::ostringstream msg;
	    msg << "Checking inv_mass_inertia for test_index " << test_index
		<< " q = " << state.position_ << "\n";
	    pretty_print(MMinv_check, msg, "  want", "    ");
	    pretty_print(MMinv, msg, "  have", "    ");
	    EXPECT_TRUE (check_matrix("inv_mass_inertia", MMinv_check, MMinv, 1e-3, msg)) << msg.str();
	  }
	}
      }
    }
    catch (std::exception const & ee) {
      ADD_FAILURE () << "exception " << ee.what();
    }
    delete model;
  }
}


TEST (jspaceModel, mass_inertia_5R_nonzero)
{
  minitao::Model * model(0);
  try {
    model = create_unit_mass_5R_model();
    minitao::State state(5, 5, 0);
    minitao::zero(state.velocity_);
    
    for (size_t ii(0); ii < 5; ++ii) {
      for (double qq(-M_PI); qq <= M_PI; qq += 2 * M_PI / 7) {
	memset(&state.position_[0], 0, 5 * sizeof(double));
	state.position_[ii] = qq;
	model->update(state);
	
	minitao::Matrix MM;
	model->getMassInertia(MM);
	std::ostringstream msg;
	bool ok(true);
	for (size_t jj(0); jj < 5; ++jj) {
	  if (MM.coeff(jj, jj) < 1e-3) {
	    ok = false;
	    msg << "  MM[" << jj << "][" << jj << "] = " << MM.coeff(jj, jj) << "\n";
	  }
	}
	EXPECT_TRUE (ok) << "diagonal element below threshold\n" << msg.str();
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


TEST (jspaceModel, mass_inertia_RP)
{
  minitao::Model * model(0);
  try {
    model = create_unit_mass_RP_model();
    taoDNode * n1(model->getNode(0));
    taoDNode * n2(model->getNode(1));
    ASSERT_NE ((void*)0, n1);
    ASSERT_NE ((void*)0, n2);
    minitao::State state(2, 2, 0);
    minitao::zero(state.velocity_);
    
    for (double q1(-M_PI); q1 <= M_PI; q1 += 2 * M_PI / 7) {
      for (double q2(-1); q2 <= 1; q2 += 2.0 / 7) {
 	state.position_[0] = q1;
 	state.position_[1] = q2;
	model->update(state);
	
	double const c1(cos(q1));
	double const s1(sin(q1));
	
	minitao::Matrix MM(2, 2);
	model->getMassInertia(MM);
	minitao::Matrix MM_check(2, 2);
	MM_check.coeffRef(0, 0) = 1 + pow(s1 + c1 * q2, 2) + pow(c1 - s1 * q2, 2);
	MM_check.coeffRef(1, 0) = s1 * (s1 + c1 * q2) + c1 * (c1 - s1 * q2);
	MM_check.coeffRef(0, 1) = MM_check.coeff(1, 0);
	MM_check.coeffRef(1, 1) = 1;
	{
	  std::ostringstream msg;
	  msg << "Checking mass_inertia for q = " << state.position_ << "\n";
	  pretty_print(MM_check, msg, "  want", "    ");
	  pretty_print(MM, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("mass_inertia", MM_check, MM, 1e-3, msg)) << msg.str();
	}
	
	minitao::Matrix MMinv(2, 2);
	model->getInverseMassInertia(MMinv);
	{
	  minitao::Matrix id_check(2, 2);
	  id_check.setIdentity();
	  minitao::Matrix id;
	  id = MM * MMinv;
	  std::ostringstream msg;
	  msg << "Checking A * Ainv = I for q = " << state.position_ << "\n";
	  pretty_print(id_check, msg, "  want", "    ");
	  pretty_print(id, msg, "  have", "    ");
	  EXPECT_TRUE (check_matrix("identity", id_check, id, 1e-3, msg)) << msg.str();
	}
      }
    }
  }
  catch (std::exception const & ee) {
    ADD_FAILURE () << "exception " << ee.what();
  }
  delete model;
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS ();
}


std::string create_puma_frames() throw(runtime_error)
{
  static char const * frames = 
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0.7854	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0.382684  0.923879 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { -0.172181  0.17218  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.199191  0.411466  -2.77556e-17 }\n"
    "  ID 3: r: { 0  0  0.382684  0.923879 }  t: { 0.184837  0.397112  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.184837  0.397112  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.184837  0.397112  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0.7854	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.305328  0.1501  -0.305329 }\n"
    "  ID 3: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.597222  0.1501  0.0152724 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0.7854	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.723694  0.1501  0.320602 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.723694  0.1501  0.320602 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.723694  0.1501  0.320602 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0.7854	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0.7854	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0.382684  8.32667e-17  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	0.7854	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	-0.7854	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  -0.382684  0.923879 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.172181  0.17218  0 }\n"
    "  ID 2: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.411465  -0.199193  -2.77556e-17 }\n"
    "  ID 3: r: { 0  0  -0.382684  0.923879 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.397111  -0.184838  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	-0.7854	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.305328  0.1501  0.305329 }\n"
    "  ID 3: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { -0.0152746  0.1501  0.597222 }\n"
    "==================================================\n"
    "joint_positions:	0	0	-0.7854	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.111197  0.1501  0.291893 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.111197  0.1501  0.291893 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.111197  0.1501  0.291893 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	-0.7854	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  0.270599  -0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	-0.7854	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.653281  -0.270599  -0.270599  0.653281 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  -0.382684  -8.32667e-17  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	-0.7854	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.382684  0.923879 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	1.5708	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0.707108  0.707105 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.2435  -8.94425e-07  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.150102  0.431799  -5.55112e-17 }\n"
    "  ID 3: r: { 0  0  0.707108  0.707105 }  t: { -0.150102  0.411499  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { -0.150102  0.411499  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { -0.150102  0.411499  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	1.5708	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { -1.58609e-06  0.1501  -0.4318 }\n"
    "  ID 3: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.433098  0.1501  -0.411502 }\n"
    "==================================================\n"
    "joint_positions:	0	0	1.5708	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.8649  0.1501  0.0202984 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	1.5708	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	1.5708	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0.707108  1.11022e-16  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	1.5708	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	-1.5708	0	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  -0.707108  0.707105 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.2435  -8.94425e-07  0 }\n"
    "  ID 2: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.150098  -0.431801  2.77556e-17 }\n"
    "  ID 3: r: { 0  0  -0.707108  0.707105 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.150098  -0.411501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	-1.5708	0	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -1.58609e-06  0.1501  0.4318 }\n"
    "  ID 3: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.433102  0.1501  0.411498 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -0.433102  0.1501  0.411498 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.433102  0.1501  0.411498 }\n"
    "==================================================\n"
    "joint_positions:	0	0	-1.5708	0	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { -0.00129993  0.1501  -0.0203016 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	-1.5708	0	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  0.500001  -0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	-1.5708	0	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.499999  -0.500001  -0.500001  0.499999 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  -0.707108  -1.11022e-16  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n"
    "==================================================\n"
    "joint_positions:	0	0	0	0	0	-1.5708	\n"
    "link_origins:\n"
    "  ID 0: r: { 0  0  0  1 }  t: { 0  0  0 }\n"
    "  ID 1: r: { -0.707107  0  0  0.707107 }  t: { 0  0.2435  0 }\n"
    "  ID 2: r: { -0.707107  0  0  0.707107 }  t: { 0.4318  0.1501  -2.0739e-17 }\n"
    "  ID 3: r: { 0  0  0  1 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 4: r: { -0.707107  0  0  0.707107 }  t: { 0.4115  0.1501  0.4331 }\n"
    "  ID 5: r: { 0  0  -0.707108  0.707105 }  t: { 0.4115  0.1501  0.4331 }\n";
  std::string result(create_tmpfile("puma.frames.XXXXXX", frames));
  return result;
}
