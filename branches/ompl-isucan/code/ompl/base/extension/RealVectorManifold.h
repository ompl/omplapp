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
	
	/** \brief The definition of a state in R^n */
	class RealVectorState : public base::State
	{
	public:
	    double *values;
	};
	
	/** \brief The lower and upper bounds for an R^n manifold */
	typedef std::pair< std::vector<double>, std::vector<double> > RealVectorBounds;
	
	/** \brief Uniform sampler for the R^n manifold */
	class RealVectorStateUniformSampler : public base::StateSampler
	{
	public:
	    
	    RealVectorStateUniformSampler(const base::Manifold *manifold);
	    
	    virtual void sample(base::State *state);
	    virtual void sampleNear(base::State *state, const base::State *near, const double distance);
	    
	protected:
	    
	    const RealVectorBounds &m_bounds;
	    
	};
	
	/** \brief A manifold representing R^n. The distance function is the square of L2 and sampling is uniform */
	class RealVectorManifold : public base::Manifold
	{
	public:
	    
	    RealVectorManifold(unsigned int dim) : base::Manifold(), m_dimension(dim), m_stateBytes(dim * sizeof(double))
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
	    virtual unsigned int getDimension(void) const;
	    
	    /** \brief Bring the state within the bounds of the state space */
	    virtual void enforceBounds(base::State *state) const;
	    	    
	    /** \brief Check if a state is inside the bounding box */
	    virtual bool satisfiesBounds(const base::State *state) const;
	    
	    /** \brief Copy a state to another */
	    virtual void copyState(base::State *destination, const base::State *source) const;
	    
	    /** \brief Computes distance to between two states */
	    virtual double distance(const base::State *state1, const base::State *state2) const;
	    
	    /** \brief Checks whether two states are equal */
	    virtual bool equalStates(const base::State *state1, const base::State *state2) const;

	    /** \brief Computes the state that lies at time t \in [0, 1] on the
		segment that connects the current state to the
		destination state */
	    virtual void interpolate(const base::State *from, const base::State *to, const double t, base::State *state) const;

	    /** \brief Allocate an instance of a state sampler for this space */
	    virtual base::StateSamplerPtr allocStateSampler(void) const;
	    
	    /** \brief Allocate a state that can store a point in the described space */
	    virtual base::State* allocState(void) const;
	    
	    /** \brief Free the memory of the allocated state */
	    virtual void freeState(base::State *state) const;

	    /** \brief Print a state to screen */
	    virtual void printState(const base::State *state, std::ostream &out) const;

	protected:
	    
	    unsigned int     m_dimension;
	    std::size_t      m_stateBytes;
	    RealVectorBounds m_bounds;

	};
    }
}

#endif
