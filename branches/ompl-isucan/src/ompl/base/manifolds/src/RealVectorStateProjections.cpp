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

#include "ompl/base/manifolds/RealVectorStateProjections.h"
#include "ompl/util/Exception.h"

ompl::base::RealVectorLinearProjectionEvaluator::RealVectorLinearProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<double> &cellDimensions,
										     const std::vector< std::valarray<double> > &projection) : ProjectionEvaluator(manifold, cellDimensions), projection_(projection)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_.get()))
	throw Exception("Expected real vector manifold for projection");
}

ompl::base::RealVectorOrthogonalProjectionEvaluator::RealVectorOrthogonalProjectionEvaluator(const StateManifoldPtr &manifold, const std::vector<double> &cellDimensions,
											     const std::vector<unsigned int> &components) : ProjectionEvaluator(manifold, cellDimensions), components_(components)
{
    if (!dynamic_cast<const RealVectorStateManifold*>(manifold_.get()))
	throw Exception("Expected real vector manifold for projection");
}

void ompl::base::RealVectorLinearProjectionEvaluator::project(const State *state, EuclideanProjection *projection) const
{
    for (unsigned int i = 0 ; i < projection_.size() ; ++i)
    {
	const std::valarray<double> &vec = projection_[i];
	const unsigned int dim = vec.size();
	double *pos = projection + i;
	*pos = 0.0;
	for (unsigned int j = 0 ; j < dim ; ++j)
	    *pos += state->as<RealVectorState>()->values[j] * vec[j];
    }
}

void ompl::base::RealVectorOrthogonalProjectionEvaluator::project(const State *state, EuclideanProjection *projection) const
{
    for (unsigned int i = 0 ; i < components_.size() ; ++i)
	projection[i] = state->as<RealVectorState>()->values[components_[i]];
}
