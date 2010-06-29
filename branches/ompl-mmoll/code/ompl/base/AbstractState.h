/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of Rice University nor the names of its
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

/* \author Ioan Sucan, Mark Moll */

#ifndef OMPL_BASE_ABSTRACTSTATE_
#define OMPL_BASE_ABSTRACTSTATE_

#include "ompl/base/General.h"

namespace ompl
{
	namespace base
	{

	/// Negligible difference between state components
	template<class T>
	struct epsilon
	{
		static const T value;
	};
	template<>
	struct epsilon<double>
	{
		static const double value = 1e-6;
	};
	template<>
	struct epsilon<int>
	{
		static const int value = 0;
	};
	
	/// Abstract base class for state classes
	class AbstractState
	{
	public:
		virtual AbstractState* clone() = 0;
		static std::size_t size();
		virtual double distance(const AbstractState&) const = 0;
		// virtual AbstractState& invert() = 0;
		// virtual AbstractState& operator+=(const AbstractState&) = 0;
		// virtual AbstractState& operator-=(const AbstractState&) = 0;
		virtual bool operator==(const AbstractState&) const = 0;
		virtual void interpolate(const AbstractState&, const double, AbstractState&) const = 0;
		virtual bool satisfiesBounds(const AbstractState&, const AbstractState&) const = 0;
		virtual AbstractState& enforceBounds(const AbstractState&, const AbstractState&) = 0;
	};

	}
}
#endif
