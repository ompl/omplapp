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

#include "ompl/control/SpaceInformation.h"
#include "ompl/util/Exception.h"
#include <limits>

void ompl::control::SpaceInformation::setup(void)
{
    base::SpaceInformation::setup();
    if (m_minSteps > m_maxSteps)
	throw Exception("The minimum number of steps cannot be larger than the maximum number of steps");
    if (m_minSteps == 0 && m_maxSteps == 0)
    {
	m_minSteps = 1;
	m_maxSteps = 2;
	m_msg.warn("Assuming propagation will always have 1 or 2 steps");
    }
    if (m_minSteps < 1)
	throw Exception("The minimum number of steps must be at least 1");
    if (m_stepSize < std::numeric_limits<double>::round_error())
    {
	m_stepSize = m_resolution;
	m_msg.warn("The propagation step size is assumed to be the same as the state validity checking resolution: %f", m_stepSize);
    }
}

unsigned int ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, base::State *result, bool stopBeforeInvalid) const
{
}

unsigned int ompl::control::SpaceInformation::propagate(const base::State *state, const Control* control, unsigned int steps, std::vector<base::State*> &result, bool stopBeforeInvalid, bool alloc) const
{
}

void ompl::control::SpaceInformation::printSettings(std::ostream &out) const
{
    base::SpaceInformation::printSettings(out);
    out << "  - control manifold:" << std::endl;
    m_controlManifold->printSettings(out);
    out << "  - propagation step size: " << m_stepSize << std::endl;
    out << "  - propagation duration: [" << m_minSteps << ", " << m_maxSteps << "]" << std::endl;
}

