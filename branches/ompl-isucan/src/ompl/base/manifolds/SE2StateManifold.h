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

/* Author: Ioan Sucan */

#ifndef OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_
#define OMPL_BASE_MANIFOLDS_SE2_STATE_MANIFOLD_

#include "ompl/base/StateManifold.h"
#include "ompl/base/manifolds/RealVectorStateManifold.h"
#include "ompl/base/manifolds/SO2StateManifold.h"
#include "ompl/base/ScopedState.h"

namespace ompl
{
    namespace base
    {
		
	/** \brief A manifold representing SE(2) */
	class SE2StateManifold : public CompoundStateManifold
	{
	public:

	    /** \brief Interface to an SE(2) state */
	    class Mapper
	    {
	    public:
		Mapper(State *state) : state_(state)
		{
		}
		
		template<class T>
		Mapper(const ScopedState<T> &state) : state_(state.get())
		{
		}
		
		void use(State *state)
		{
		    state_ = state;
		}
		
		template<class T>
		void use(const ScopedState<T> &state)
		{
		    state_ = state.get();
		}
		
		double getX(void) const
		{
		    return state_->as<CompoundState>()->as<RealVectorStateManifold::StateType>(0)->values[0];
		}
		
		double getY(void) const
		{
		    return state_->as<CompoundState>()->as<RealVectorStateManifold::StateType>(0)->values[1];
		}
		
		double getYaw(void) const
		{
		    return state_->as<CompoundState>()->as<SO2StateManifold::StateType>(1)->value;
		}
		
		void setX(double x)
		{
		    state_->as<CompoundState>()->as<RealVectorStateManifold::StateType>(0)->values[0] = x;
		}
		
		void setY(double y)
		{
		    state_->as<CompoundState>()->as<RealVectorStateManifold::StateType>(0)->values[1] = y;
		}
		
		void setYaw(double yaw)
		{
		    state_->as<CompoundState>()->as<SO2StateManifold::StateType>(1)->value = yaw;
		}
		
	    private:
		
		State *state_;
		
	    };
	    
	    SE2StateManifold(void) : CompoundStateManifold()
	    {
		addSubManifold(StateManifoldPtr(new RealVectorStateManifold(2)), 1.0);
		addSubManifold(StateManifoldPtr(new SO2StateManifold()), 0.5);
		lock();
	    }
	    
	    virtual ~SE2StateManifold(void)
	    {	
	    }	
	    
	    /** \copydoc RealVectorStateManifold::setBounds() */
	    void setBounds(const RealVectorBounds &bounds)
	    {
		as<RealVectorStateManifold>(0)->setBounds(bounds);
	    }
	    
	    /** \copydoc RealVectorStateManifold::getBounds() */
	    const RealVectorBounds& getBounds(void) const
	    {
		return as<RealVectorStateManifold>(0)->getBounds();
	    }    

	    virtual void setup(void);
	    
	};	
    }
}

#endif
