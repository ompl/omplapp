/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* \author Ioan Sucan */

#ifndef OMPL_BASE_EXTENSION_REAL_VECTOR_MANIFOLD_
#define OMPL_BASE_EXTENSION_REAL_VECTOR_MANIFOLD_

#include "ompl/base/Manifold.h"
#include <vector>
#include <utility>

namespace ompl
{
    namespace ext
    {
	
	class RealVectorState : public base::State
	{
	public:
	    double *values;
	};
	
	typedef std::pair< std::vector<double>, std::vector<double> > RealVectorBounds;
	
	class RealVectorStateSampler : public base::StateSampler
	{
	public:
	    
	    RealVectorStateSampler(const ManifoldConstPtr &si);
	    
	    virtual void sample(State *state);	    
	    virtual void sampleNear(State *state, const State *near, const double distance);
	    
	protected:
	    
	    const RealVectorBounds &m_bounds;
	    
	};
	
	class RealVectorManifold : public base:Manifold
	{
	public:
	    
	    RealVectorManifold(unsigned int dim) : base::Manifold(), m_dimension(dim)
	    {
	    }
	    
	    virtual ~RealVectorManifold(void)
	    {	
	    }
	    
	    void setBounds(const RealVectorBounds &bounds)
	    {
		m_bounds = bounds;
	    }
	    
	    const RealVectorBounds& getBounds(void) const
	    {
		return m_bounds;
	    }
	    
	    /** \brief Get the dimension of the space */
	    virtual unsigned int getDimension(void) const
	    {
		return m_dimension;
	    }
	    
	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(State *state) const
	    {
		RealVectorState *rstate = static_cast<RealVectorState*>(state);
		
	    }	    
	    	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const State *state) const;
	    
	    /** \brief Copy a state to another */
	    virtual void copyState(State *destination, const State *source) const = 0;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const State *state1, const State *state2) const = 0;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const State *state1, const State *state2) const = 0;

	    /** \brief Computes the state that lies at time t \in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const State *from, const State *to, const double t, State *state) const = 0;

	    /** \brief Allocate an instance of a state sampler for this space */
	    virtual StateSampler* allocStateSampler(void) const = 0;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual State* allocState(void) const = 0;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(State *state) const = 0;

	    /** \brief Print a state to screen */
	    virtual void printState(const State *state, std::ostream &out) const;

	protected:
	    
	    unsigned int        m_dimension;
	    std::vector<double> m_lowerBound;
	    std::vector<double> m_upperBound;
	    
	};
    }
}

#endif
