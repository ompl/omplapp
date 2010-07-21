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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_SQRT_APPROX_

#include "ompl/datastructures/NearestNeighbors.h"
#include <cmath>

namespace ompl
{

    template<typename _T>
    class NearestNeighborsSqrtApprox : public NearestNeighbors<_T>
    {
    public:
        NearestNeighborsSqrtApprox(void) : NearestNeighbors<_T>()
	{
	    checks_ = 0;
	}
	
	virtual ~NearestNeighborsSqrtApprox(void)
	{
	}
	
	virtual void clear(void)
	{
	    data_.clear();
	    active_.clear();
	}

	virtual void add(_T &data)
	{
	    data_.push_back(data);
	    active_.push_back(true);
	    checks_ = 1 + (int)floor(sqrt(data_.size()));
	}

	virtual bool remove(_T &data)
	{
	    for (int i = data_.size() - 1 ; i >= 0 ; --i)
		if (data_[i] == data)
		{
		    active_[i] = false;
		    return true;
		}
	    return false;
	}
	
	virtual _T nearest(const _T &data) const
	{
	    int pos = -1;
	    if (checks_ > 0)
	    {
		double dmin = 0.0;
		unsigned int n = data_.size();
		unsigned int offset = reinterpret_cast<unsigned long>(&data) % checks_;
		for (unsigned int j = 0 ; j < checks_ ; ++j)
		{
		    unsigned int i = (j * checks_ + offset) % n;
		    unsigned int c = 0;
		    while (!active_[i] && c < n)
		    {
			i = (i + 1) % n;
			c++;
		    }	    
		    
		    if (active_[i])
		    {
			double distance = NearestNeighbors<_T>::distFun_(data_[i], data);
			if (pos < 0 || dmin > distance)
			{
			    pos = i;
			    dmin = distance;
			}
		    }
		}
	    }
	    
	    return pos >= 0 ? data_[pos] : data;
	}
	
	virtual unsigned int size(void) const
	{
	    return data_.size();
	}
	
	virtual void list(std::vector<_T> &data) const
	{
	    data = data_;
	}
	
    protected:
	
	std::vector<_T>   data_;
	std::vector<bool> active_;
	unsigned int      checks_;
    };
    
    
}

#endif
