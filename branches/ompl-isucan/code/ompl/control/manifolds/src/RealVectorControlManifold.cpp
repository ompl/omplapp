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

#include "ompl/control/manifolds/RealVectorControlManifold.h"
#include "ompl/util/Exception.h"
#include <cstring>
#include <limits>

void ompl::control::RealVectorControlUniformSampler::sample(Control *control)
{
    const unsigned int dim = m_manifold->getDimension();
    const RealVectorBounds &bounds = static_cast<const RealVectorControlManifold*>(m_manifold)->getBounds();
    
    RealVectorControl *rcontrol = static_cast<RealVectorControl*>(control);
    for (unsigned int i = 0 ; i < dim ; ++i)
	rcontrol->values[i] = m_rng.uniformReal(bounds.low[i], bounds.high[i]);
}

void ompl::control::RealVectorControlManifold::setBounds(const RealVectorBounds &bounds)
{
    if (bounds.low.size() != bounds.high.size())
	throw Exception("Lower and upper bounds are not of same dimension");
    if (bounds.low.size() != m_dimension)
	throw Exception("Bounds do not match dimension of manifold");
    m_bounds = bounds;
}

unsigned int ompl::control::RealVectorControlManifold::getDimension(void) const
{
    return m_dimension;
}

void ompl::control::RealVectorControlManifold::copyControl(Control *destination, const Control *source) const
{
    memcpy(static_cast<RealVectorControl*>(destination)->values,
	   static_cast<const RealVectorControl*>(source)->values, m_controlBytes);    
}

bool ompl::control::RealVectorControlManifold::equalControls(const Control *control1, const Control *control2) const
{
    const double *s1 = static_cast<const RealVectorControl*>(control1)->values;
    const double *s2 = static_cast<const RealVectorControl*>(control2)->values;
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
    {	 
	double diff = (*s1++) - (*s2++);
	if (fabs(diff) > std::numeric_limits<double>::epsilon())
	    return false;
    }
    return true;
}

ompl::control::ControlSamplerPtr ompl::control::RealVectorControlManifold::allocControlSampler(void) const 
{
    return ControlSamplerPtr(new RealVectorControlUniformSampler(this));
}

ompl::control::Control* ompl::control::RealVectorControlManifold::allocControl(void) const
{
    RealVectorControl *rcontrol = new RealVectorControl();
    rcontrol->values = new double[m_dimension];
    return rcontrol;
}

void ompl::control::RealVectorControlManifold::freeControl(Control *control) const
{
    RealVectorControl *rcontrol = static_cast<RealVectorControl*>(control);
    delete[] rcontrol->values;
    delete rcontrol;
}

void ompl::control::RealVectorControlManifold::nullControl(Control *control) const
{
    RealVectorControl *rcontrol = static_cast<RealVectorControl*>(control);
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
	rcontrol->values[i] = 0.0;
}

void ompl::control::RealVectorControlManifold::printControl(const Control *control, std::ostream &out) const
{
    if (control)
    {
	const RealVectorControl *rcontrol = static_cast<const RealVectorControl*>(control);
	for (unsigned int i = 0 ; i < m_dimension ; ++i)
	    out << rcontrol->values[i] << " ";
	out << std::endl;
    }
    else
	out << "NULL" << std::endl;
}

void ompl::control::RealVectorControlManifold::printSettings(std::ostream &out) const
{
    out << "Real vector control manifold with bounds: " << std::endl;
    out << "  - min: ";
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
	out << m_bounds.low[i] << " ";
    out << std::endl;    
    out << "  - max: ";
    for (unsigned int i = 0 ; i < m_dimension ; ++i)
	out << m_bounds.high[i] << " ";
    out << std::endl;
}
