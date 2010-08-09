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
*   * Neither the name of the Rice University nor the names of its
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

#ifndef OMPL_BASE_PROJECTION_EVALUATOR_CONTAINER_
#define OMPL_BASE_PROJECTION_EVALUATOR_CONTAINER_

#include "ompl/base/ProjectionEvaluator.h"
#include "ompl/util/ClassForward.h"
#include <boost/noncopyable.hpp>
#include <string>

namespace ompl
{
    
    namespace base
    {
	
	ClassForward(SpaceInformation);
	
	class ProjectionEvaluatorContainer : private boost::noncopyable
	{
	public:
	    
	    ProjectionEvaluatorContainer(const SpaceInformationPtr &si, const std::string &prefix = "") : si_(si.get()), msg_(prefix)
	    {
	    }
	    
	    ProjectionEvaluatorContainer(const SpaceInformation *si, const std::string &prefix = "") : si_(si), msg_(prefix)
	    {
	    }
	    
	    ~ProjectionEvaluatorContainer(void)
	    {
	    }
	    
	    ProjectionEvaluator& operator*(void) const
	    {
		return *proj_;
	    }

	    ProjectionEvaluator* operator->(void) const
	    {
		return proj_.get();
	    }

	    ProjectionEvaluator* get(void) const
	    {
		return proj_.get();
	    }
	    
	    void set(const std::string &projectionName);
	    void set(const ProjectionEvaluatorPtr &proj);
	    
	    void setup(void);
	    
	private:
	    
	    const SpaceInformation *si_;
	    msg::Interface          msg_;
	    ProjectionEvaluatorPtr  proj_;
	};
    }
}

#endif
