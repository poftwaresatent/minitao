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
   \file Model.hpp
   \author Roland Philippsen, inspired by wbc/core code of Luis Sentis
*/

#ifndef MINITAO_MODEL_HPP
#define MINITAO_MODEL_HPP

#include "State.hpp"
#include "tao_util.hpp"
#include "wrap_eigen.hpp"
#include <string>
#include <vector>
#include <set>


namespace minitao {
  
  
  class Model
  {
  public:
    Model(/** TAO tree used for computing kinematics, the gravity
	      torque vector, the mass-inertia matrix, and its
	      inverse. */
	  taoDNode * kgm_root,
	  /** Optional TAO tree for computing Coriolis and
	      centrifugal torques. If you set this to NULL, then the
	      Coriolis and centrifugal forces won't be computed. */
	  taoDNode * cc_root);
    
    ~Model();
    
    //////////////////////////////////////////////////
    // fire-and-forget facet
    
    /** Calls setState(), updateKinematics(), and
	updateDynamics(). After calling the update() method, you can
	use any of the other methods without worrying whether you have
	already called the corresponding computeFoo() method. */
    void update(State const & state);
    
    /** Inform the model about the joint state. We have to separate
	the state update from the computation of the various
	quantities in order to efficiently use the TAO tree
	representation, which forces us to distribute the state over
	its nodes before computing the model.
	
	\pre The NDOF of the state has to match the NDOF of the
	model. No check is performed in this method, it can crash your
	program if you're not careful.
    */
    void setState(State const & state);
    
    /** Retrieve the state passed to setState() (or update(), for that
	matter). */
    inline State const & getState() const { return state_; }
    
    //////////////////////////////////////////////////
    // Bare tree accessors.
    
    /** Compute or retrieve the cached number of nodes in the
	robot.
	
	\note
	- the root node is NOT included in this count.
	- in principle, each node can have any number of joints, and
	  each joint can have any number of degrees of freedom, which
	  is why getNJoints() and getNDOF() might come in handy, too. */
    size_t getNNodes() const;
    
    /** Compute or retrieve the cached number of joints in the
	robot. Note that each joint can have any number of degrees of
	freedom, which is why getNDOF() might come in handy, too. */
    size_t getNJoints() const;
    
    /** Compute or retrieve the cached number of degrees of freedom of
	the robot. Note that each joint can have any number of degrees
	of freedom, which is why this method might return something
	else than getNJoints(). */
    size_t getNDOF() const;
    
    /** Deprecated TAO node access method which ends up doing a linear
	(order NDOF) search over the KGM tree.
	
	\note TAO node IDs are not a particularly good method of
	indexing branches of your robot...
	
	\return The first node that matches the ID, or NULL if the id
	was not found in the KGM tree.
    */
    taoDNode * findNodeByID(int id) const;
    
    //////////////////////////////////////////////////
    // kinematic facet
    
    /** Computes the node origins wrt the global frame. */
    void updateKinematics();
    
    /** Retrieve the frame (translation and rotation) of a node
	origin.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool getGlobalFrame(taoDNode const * node,
			Transform & global_transform) const;
    
    /** Compute the global frame (translation and rotation)
	corresponding to a local frame expressed wrt the origin of a
	given node.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    Transform const & local_transform,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in three
	doubles.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    double local_x, double local_y, double local_z,
			    Transform & global_transform) const;
    
    /** Convenience method in case you are only interested in the
	translational part and hold the local point in a
	three-dimensional vector.
	
	\return True on success. The only possible failure stems from
	an invalid node, so if you got that using getNode() or one of
	the related methods you can safely ignore the return value. */
    bool computeGlobalFrame(taoDNode const * node,
			    Vector const & local_translation,
			    Transform & global_transform) const;
    
    /** Compute the Jacobian (J_v over J_omega) at the origin of a
	given node.
	
	\note This just ends up calling the other computeJacobian()
	which takes a global point as argument, passing in the origin
	of the given node.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 Matrix & jacobian) const;
    
    /** Compute the Jacobian (J_v over J_omega) for a given node, at a
	point expressed wrt to the global frame.
	
	\todo Implement support for more than one joint per node, and
	more than one DOF per joint.
	
	\return True on success. There are two possible failures: an
	invalid node, or an unsupported joint type. If you got the
	node using getNode() or one of the related methods, then you
	need to extend this implementation when it returns false. */
    bool computeJacobian(taoDNode const * node,
			 double gx, double gy, double gz,
			 Matrix & jacobian) const;
    
    /** Convenience method in case you are holding the global position
	in a three-dimensional vector. */
    inline bool computeJacobian(taoDNode const * node,
				Vector const & global_point,
				Matrix & jacobian) const
    { return computeJacobian(node, global_point[0], global_point[1], global_point[2], jacobian); }
    
    //////////////////////////////////////////////////
    // dynamics facet
    
    /** Calls computeGravity(), computeCoriolisCentrifugal(),
	computeMassInertia(), and computeInverseMassInertia(). */
    void updateDynamics();
    
    /** Compute the gravity joint-torque vector. */
    void computeGravity();
    
    /** Disable (or enable) gravity compensation for a given DOF
	(specified using its index). If you set \c disable to \c true,
	then getGravity() will knock out (set to zero) the
	corresponding entry of the gravity joint-torque vector.
	
	\note Invalid indices are silently ignore, and \c true is
	returned. Valid indices are \c 0<=index<getNDOF().
	
	\return The previous value of \c disable for this joint. */
    bool disableGravityCompensation(size_t index, bool disable);
    
    /** Retrieve the gravity joint-torque vector.
	
	\return True on success. The only possibility of receiving
	false is if you never called updateDynamics(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getGravity(Vector & gravity) const;
    
    /** Compute the Coriolis and contrifugal joint-torque vector. If
	you set cc_root=NULL in the constructor, then this is a
	no-op. */
    void computeCoriolisCentrifugal();
    
    /** Retrieve the Coriolis and contrifugal joint-torque vector.
	
	\return True on success. There are two possibility of
	receiving false: (i) you set cc_root=NULL in the constructor,
	or (ii) you never called computeCoriolisCentrifugal(), which
	gets called by updateDynamics(), which gets called by
	update(). */
    bool getCoriolisCentrifugal(Vector & coriolis_centrifugal) const;
    
    /** Compute the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix. */
    void computeMassInertia();
    
    /** Retrieve the joint-space mass-inertia matrix, a.k.a. the
	kinetic energy matrix.
	
	\return True on success. The only possibility of receiving
	false is if you never called computeMassInertia(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getMassInertia(Matrix & mass_inertia) const;
    
    /** Compute the inverse joint-space mass-inertia matrix. */
    void computeInverseMassInertia();
    
    /** Retrieve the inverse joint-space mass-inertia matrix. 
	
	\return True on success. The only possibility of receiving
	false is if you never called computeMassInertia(), which gets
	called by updateDynamics(), which gets called by update(). */
    bool getInverseMassInertia(Matrix & inverse_mass_inertia) const;
    
    
    /** For debugging only, access to the
	kinematics-gravity-mass-inertia tree. */
    taoDNode * _getKGMRoot() { return kgm_root_; }
    
    /** For debugging only, access to the optional
	Coriolis-centrifugal tree. Can be NULL if the user is not
	interested in Coriolis-centrifugal effects. */
    taoDNode * _getCCRoot() { return cc_root_; }
    
    
  private:
    typedef std::set<size_t> dof_set_t;
    dof_set_t gravity_disabled_;
    
    std::size_t ndof_;
    taoDNode * kgm_root_;
    nodeVector_t kgm_nodes_;
    jointVector_t kgm_joints_;
    
    taoDNode * cc_root_;
    nodeVector_t cc_nodes_;
    jointVector_t cc_joints_;
    
    State state_;
    std::vector<double> g_torque_;
    std::vector<double> cc_torque_;
    std::vector<double> a_upper_triangular_;
    std::vector<double> ainv_upper_triangular_;
  };
  
}

#endif // MINITAO_MODEL_HPP
