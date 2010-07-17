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

/** \author Ioan Sucan */

#ifndef OMPL_CONTROL_SPACE_INFORMATION_
#define OMPL_CONTROL_SPACE_INFORMATION_

#include "ompl/base/SpaceInformation.h"
#include "ompl/control/ControlManifold.h"
#include "ompl/control/ControlSampler.h"
#include "ompl/control/Control.h"
#include "ompl/util/ClassForward.h"

namespace ompl
{
    
    /** \brief This namespace contains sampling based planning
	routines used by planning under differential constraints */
    namespace control
    {

	ClassForward(SpaceInformation);
	
	class SpaceInformation : public base::SpaceInformation
	{
	public:
	    
	    /** \brief Constructor. Sets the instance of the manifold
		to plan on. */
	    SpaceInformation(const base::StateManifoldPtr &stateManifold, const ControlManifoldPtr &controlManifold) : base::SpaceInformation(stateManifold),
														       m_controlManifold(controlManifold),
														       m_minSteps(0), m_maxSteps(0), m_stepSize(0.0)
	    {
	    }
	    	    
	    virtual ~SpaceInformation(void)
	    {
	    }
	    
	    /** \brief Allocate memory for a control */
	    Control* allocControl(void) const
	    {
		return m_controlManifold->allocControl();
	    }
	    
	    /** \brief Free the memory of a control */
	    void freeControl(Control *control) const
	    {
		m_controlManifold->freeControl(control);
	    }

	    /** \brief Print a control to a stream */
	    void printControl(const Control *control, std::ostream &out = std::cout) const
	    {
		m_controlManifold->printControl(control, out);
	    }

	    /** \brief Copy a control to another */
	    void copyControl(Control *destination, const Control *source) const
	    {
		m_controlManifold->copyControl(destination, source);
	    }
	    
	    /** \brief Clone a control */
	    Control* cloneControl(const Control *source) const
	    {
		Control *copy = m_controlManifold->allocControl();
		m_controlManifold->copyControl(copy, source);
		return copy;
	    }
	    
	    /** \brief Check if two controls are the same */
	    bool equalControls(const Control *control1, const Control *control2) const
	    {
		return m_controlManifold->equalControls(control1, control2);
	    }
	    
	    /** \brief Make the control have no effect if it were to be applied to a state for any amount of time. */
	    void nullControl(Control *control) const
	    {
		m_controlManifold->nullControl(control);
	    }
	    
	    /** \brief Allocate a control sampler */
	    ControlSamplerPtr allocControlSampler(void) const
	    {
		return m_controlManifold->allocControlSampler();
	    }
	    
	    /** \brief When controls are applied to states, they are applied for a time duration that is an integer
		multiple of the stepSize, within the bounds specified by setMinMaxControlDuration() */
	    void setPropagationStepSize(double stepSize)
	    {
		m_stepSize = stepSize;
	    }
	    
	    double getPropagationStepSize(void) const
	    {
		return m_stepSize;
	    }

	    /** \brief Set the minimum and maximum number of steps a control is propagated for */
	    void setMinMaxControlDuration(unsigned int minSteps, unsigned int maxSteps)
	    {
		m_minSteps = minSteps;
		m_maxSteps = maxSteps;
	    }
	    
	    /** \brief Get the minimum number of steps a control is propagated for */
	    unsigned int getMinControlDuration(void) const
	    {
		return m_minSteps;
	    }

	    /** \brief Get the maximum number of steps a control is propagated for */
	    unsigned int getMaxControlDuration(void) const
	    {
		return m_maxSteps;
	    }
	    
	    /** \brief Propagate the model of the system forward,
		starting a a given state, with a given control, for a
		given number of steps. 
		\param state the state to start at
		\param control the control to apply
		\param steps the number of time steps to apply the control for. Each time step is of length getPropagationStepSize()
		\param result the state at the end of the propagation
		\param stopBeforeInvalid if this is true, every state is checked for validity. If an invalid state is found, propagation is stopped and result is filled with the last valid state. It is assumed that the starting state is valid if stopBeforeInvalid is true
		The function returns the number of propagation steps it performed. If stopBeforeInvalid is false, this is always equal to steps. Otherwise, the return value may be less.*/
	    void propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result) const;
	    unsigned int propagateWhileValid(const base::State *state, const Control* control, unsigned int steps, base::State *result) const;
	    
	    /** \brief Same as above except that all intermediate states are returned (and optionally, memory for them is allocated) */
	    void propagate(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool includeStart, bool alloc) const;
	    unsigned int propagateWhileValid(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool includeStart, bool alloc) const;
	    
	    /** \brief Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional setup tasks (run once, before use) */
	    virtual void setup(void);
	    
	protected:
	    
	    ControlManifoldPtr m_controlManifold;
	    unsigned int       m_minSteps;
	    unsigned int       m_maxSteps;
	    double             m_stepSize;
	    
	};
	
    }
    
}
    
#endif
