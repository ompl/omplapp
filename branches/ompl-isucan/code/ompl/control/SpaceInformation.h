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

namespace ompl
{
    
    /** \brief This namespace contains sampling based planning
	routines used by planning under differential constraints */
    namespace control
    {

	class Control
	{
	protected:
	    Control(void)
	    {
	    }
	    virtual ~Control(void)
	    {
	    }
	};
	
	class CompoundControl
	{
	public:
	    Control **components;
	};
	
	// define control sampler classes
	
	enum PropagationResult
	{
	    PROPAGATION_START_VALID,
	    PROPAGATION_START_INVALID,
	    PROPAGATION_START_UNKNOWN
	};
	

	class Manifold : public base::Manifold
	{
	public:

	    /** \brief Allocate memory for a control */
	    Control* allocControl(void) const = 0;
	    
	    /** \brief Free the memory of a control */
	    void freeControl(Control *control) const = 0;

	    /** \brief Copy a control to another */
	    void copyControl(Control *destination, const Control *source) const = 0;
	    	    
	    /** \brief Check if two controls are the same */
	    bool equalControls(const Control *control1, const Control *control2) const = 0;
	    
	    void nullControl(Control *control) const = 0;
	    
	    /** \brief Allocate a control sampler */
	    ControlSamplerPtr allocControlSampler(void) const = 0;

	    /** \brief Propagate forward from a state, given a control, for some time.

		In the process of propagation, it is sometimes the case that collisions are evaluated (e.g., with physics
		simulation).  Important: This is not the same as state validity, but it may represent an important
		computational part of checking state validity. The implementation of this function may
		choose to evaluate the full validity of the starting state of the propagation. If this is the case, and the
		state is valid, the return value of the function is PROPAGATION_START_VALID. If the state is not valid,
		the return value is PROPAGATION_START_INVALID. If no such check is performed, the return value is
		PROPAGATION_START_UNKNOWN. Returning PROPAGATION_START_UNKNOWN always leads to a correct
		implementation but may not be the most efficient one. */
	    virtual PropagationResult propagate(const base::State *state, const Control* control, const double duration, base::State *result) const = 0;
	    
	    /** \brief Print a control to a stream */
	    virtual void printControl(const Control *control, std::ostream &out = std::cout) const;

	    virtual void printSettings(std::ostream &out) const;
	};
	
	class CompoundManifold : public base::CompoundManifold
	{
	public:
	    
	    Control* allocControl(void) const;
	    
	    /** \brief Free the memory of a control */
	    void freeControl(Control *control) const;

	    /** \brief Copy a control to another */
	    void copyControl(Control *destination, const Control *source) const;
	    	    
	    /** \brief Check if two controls are the same */
	    bool equalControls(const Control *control1, const Control *control2) const;
	    
	    void nullControl(Control *control) const;
	    
	    ControlSamplerPtr allocControlSampler(void) const;

	    virtual PropagationResult propagate(const base::State *state, const Control* control, const double duration, base::State *result) const;
	    
	    virtual void printControl(const Control *control, std::ostream &out = std::cout) const;

	    virtual void printSettings(std::ostream &out) const;
	};
		
	class SpaceInformation : public base::SpaceInformation
	{
	public:
	    
	    /** \brief Constructor. Sets the instance of the manifold
		to plan on. */
	    SpaceInformation(const base::ManifoldPtr &manifold) : base::SpaceInformation(manifold)
	    {
		m_cManifold = dynamic_cast<Manifold*>(m_manifold.get());
		if (!m_cManifold)
		    throw Exception("Cannot create a state space with controls without a manifold that supports controls");
	    }
	    	    
	    virtual ~SpaceInformation(void)
	    {
	    }
	    
	    /** \brief Allocate memory for a state */
	    Control* allocControl(void) const
	    {
		return m_cManifold->allocControl();
	    }
	    
	    /** \brief Free the memory of a state */
	    void freeControl(Control *state) const
	    {
		m_manifold->freeControl(state);
	    }

	    /** \brief Print a state to a stream */
	    void printControl(const Control *state, std::ostream &out = std::cout) const
	    {
		m_cManifold->printState(state, out);
	    }

	    /** \brief Copy a state to another */
	    void copyControl(State *destination, const State *source) const
	    {
		m_manifold->copyState(destination, source);
	    }
	    
	    /** \brief Clone a state */
	    State* cloneControl(const State *source) const
	    {
		State *copy = m_manifold->allocState();
		m_manifold->copyState(copy, source);
		return copy;
	    }
	    
	    /** \brief Check if two states are the same */
	    bool equalControls(const State *state1, const State *state2) const
	    {
		return m_manifold->equalStates(state1, state2);
	    }
	    
	    void nullControl(Control *control) const
	    {
	    }
	    
	    /** \brief Allocate a state sampler */
	    StateSamplerPtr allocControlSampler(void) const
	    {
		return m_manifold->allocStateSampler();
	    }
	    
	    void setPropagationStepSize(double stepSize)
	    {
		m_stepSize = stepSize;
	    }
	    
	    double getPropagationStepSize(void) const
	    {
		return m_stepSize;
	    }
	    
	    /** \brief Propagate the model of the system forward,
		starting a a given state, with a given control, for a
		given number of steps. 
		\param state the state to start at
		\param control the control to apply
		\param steps the number of time steps to apply the control for. Each time step is of length getPropagationStepSize()
		\param result the state at the end of the propagation
		\param stopBeforeInvalid if this is true, every state is checked for validity. If an invalid state is found, propagation is stopped and result is filled with the last valid state
		The function returns the number of propagation steps it performed. If stopBeforeInvalid is false, this is always equal to steps. Otherwise, the return value may be less.*/
	    unsigned int propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result, bool stopBeforeInvalid) const;
	    
	    /** \brief Same as above except that all intermediate states are returned (and optionally, memory for them is allocated) */
	    unsigned int propagate(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool stopBeforeInvalid, bool alloc) const;	    
	    
	    /** \brief Print information about the current instance of the state space */
	    virtual void printSettings(std::ostream &out = std::cout) const;
	    
	    /** \brief Perform additional setup tasks (run once, before use) */
	    virtual void setup(void);
	    
	protected:
	    
	    Manifold               *m_cManifold;
	};
	
    }
    
}
    
#endif
