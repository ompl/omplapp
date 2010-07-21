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

#ifndef OMPL_GEOMETRIC_PATH_SIMPLIFIER_
#define OMPL_GEOMETRIC_PATH_SIMPLIFIER_

#include "ompl/base/SpaceInformation.h"
#include "ompl/geometric/PathGeometric.h"
#include "ompl/util/ClassForward.h"
#include "ompl/util/RandomNumbers.h"

namespace ompl
{

    namespace geometric
    {

	ClassForward(PathSimplifier);
	
	/** \brief This class contains smoothers that can be applied to geometric paths.

	 These are in fact routines that shorten the path, and do not
	 necessarily make it smoother.*/
	class PathSimplifier
	{
	public:
	    PathSimplifier(const base::SpaceInformationPtr &si)
	    {
		si_ = si;
		rangeRatio_ = 0.2;
		maxSteps_ = 10;
		maxEmptySteps_ = 3;
	    }
	    
	    virtual ~PathSimplifier(void)
	    {
	    }
	    
	    double getRangeRatio(void) const
	    {
		return rangeRatio_;
	    }
	    
	    void setRangeRatio(double rangeRatio)
	    {
		rangeRatio_ = rangeRatio;
	    }
	    
	    unsigned int getMaxSteps(void) const
	    {
		return maxSteps_;
	    }
	    
	    void setMaxSteps(unsigned int maxSteps)
	    {
		maxSteps_ = maxSteps;
	    }
	    
	    unsigned int getMaxEmptySteps(void) const
	    {
		return maxEmptySteps_;
	    }
	    
	    void setMaxEmptySteps(unsigned int maxEmptySteps)
	    {
		maxEmptySteps_ = maxEmptySteps;
	    }
	    
	    /** \brief Given a path, attempt to remove vertices from it while keeping the path valid */
	    virtual void reduceVertices(PathGeometric &path);
	    
	    /** \brief Given a path, attempt to remove vertices from it while
	     * keeping the path valid.  Then, interpolate the path, to add
	     * more vertices and try to remove them again. This should
	     * produce smoother solutions. removeRedundantCommands is also
	     * called.  */
	    virtual void simplifyMax(PathGeometric &path);
	    
	protected:
	    
	    base::SpaceInformationPtr si_;
	    RNG                       rng_;
	    double                    rangeRatio_;
	    unsigned int              maxSteps_;
	    unsigned int              maxEmptySteps_;
	};    
    }
}

#endif
