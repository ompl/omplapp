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

#ifndef OMPL_BASE_STATEROTATION_
#define OMPL_BASE_STATEROTATION_

#include "ompl/base/State.h"

namespace ompl
{
	namespace base
	{
	/// 2D rotation
	class StateRotation2D : public State<double,1>
	{
	public:
		StateRotation2D(const double d=0.) : State<double,1>(d) {}
		StateRotation2D* clone()
		{
			return new StateRotation2D(*this);
		}
		virtual double distance(const StateRotation2D& s) const
		{
			double s0 = StateRotation2D(*this).enforceBounds()[0];
			double s1 = StateRotation2D(s).enforceBounds()[0];
			return (s0 < s1) ? std::min(s1-s0, s0-min()[0] + max()[0]-s1)
				: std::min(s0-s1, s1-min()[0] + max()[0]-s0);
		}
		virtual StateRotation2D& invert()
		{
			(*this)[0] = -(*this)[0];
			return *this;
		}
		virtual StateRotation2D& operator+=(const AbstractState& as)
		{
			(*this)[0] += static_cast<const StateRotation2D&>(as)[0];
			return *this;
		}
		virtual StateRotation2D operator+(const StateRotation2D& s) const
		{
			return StateRotation2D((*this)[0]+s[0]);
		}
		virtual StateRotation2D operator*(const StateRotation2D& s) const
		{
			return StateRotation2D((*this)[0]+s[0]);
		}
		virtual StateRotation2D& operator-=(const AbstractState& as)
		{
			const StateRotation2D& s = static_cast<const StateRotation2D&>(as);
			(*this)[0] -= s[0];
			(*this)[1] -= s[1];
			return *this;
		}
		virtual StateRotation2D operator-(const StateRotation2D& s) const
		{
			return StateRotation2D((*this)[0]-s[0]);
		}
		bool operator==(const StateRotation2D& s) const
		{
			double theta = (*this-s).enforceBounds()[0];
			return (theta < epsilon<double>::value) 
				|| (StateRotation2D::max()[0]-theta < epsilon<double>::value);
		}
		virtual StatePosition2D operator*(const StatePosition2D& p) const
		{
			double c = std::cos((*this)[0]), s = std::sin((*this)[0]);
			StatePosition2D result;
			result[0] = c * p[0] - s * p[1];
			result[1] = s * p[0] + c * p[1];
			return result;
		}
		static const StateRotation2D& min()
		{
			static const StateRotation2D sMin(0.);
			return sMin;
		}
		static const StateRotation2D& max()
		{
			static const StateRotation2D sMax(2*M_PI);
			return sMax;
		}
		virtual bool satisfiesBounds(const StateRotation2D& lo=min(), 
			const StateRotation2D& hi=max()) const
		{
			return (*this)[0] >= lo[0] && (*this)[0] <= hi[0];
		}
		virtual StateRotation2D& enforceBounds(const StateRotation2D& lo=min(), 
			const StateRotation2D& hi=max())
		{
			double tmp = std::fmod((*this)[0] - lo[0], hi[0] - lo[0]);
			tmp = tmp<0 ? tmp + hi[0] : tmp;
			(*this)[0] = tmp + lo[0];
			return *this;
		}
	};
	
	/// 3D rotation (quaternion)
	class StateRotation3D : public State<double,4>
	{
	public:
		StateRotation3D() : State<double,4>() 
		{
			(*this)[0] = 1.;
			(*this)[1] = (*this)[2] = (*this)[3] = 0.;
		}
		StateRotation3D(const double s, const double v0, const double v1, const double v2)
		{
			(*this)[0] = s;
			(*this)[1] = v0;
			(*this)[2] = v1;
			(*this)[3] = v2;
		}
		StateRotation3D* clone()
		{
			return new StateRotation3D(*this);
		}
		virtual double distance(StateRotation3D& s) const
		{
			State<double,3> t(tangent(s));
			return sqrt(t[0] * t[0] + t[1] * t[1] + t[2] * t[2]);
		}
		virtual StateRotation3D& invert()
		{
			(*this)[0] = -(*this)[0];
			return *this;
		}
		virtual StateRotation3D& operator+=(const AbstractState& as)
		{
			StateRotation3D t(*this + static_cast<const StateRotation3D&>(as));
			(*this)[0] = t[0];
			(*this)[1] = t[1];
			(*this)[2] = t[2];
			(*this)[3] = t[3];
			return *this;
		}
		virtual StateRotation3D operator+(const StateRotation3D& s) const
		{
			const StateRotation3D& t(*this);
			return StateRotation3D(
				t[0]*s[0] - t[1]*s[1] - t[2]*s[2] - t[3]*s[3],
				t[0]*s[1] + s[0]*t[1] + t[2]*s[3] - t[3]*s[2],
				t[0]*s[2] + s[0]*t[2] - t[1]*s[3] + t[3]*s[1],
				t[0]*s[3] + s[0]*t[3] + t[1]*s[2] - t[2]*s[1]
			);
		}
		virtual StateRotation3D operator*(const StateRotation3D& s) const
		{
			return *this + s;
		}
		virtual StateRotation3D& operator-=(const AbstractState& as)
		{
			const StateRotation3D& s = static_cast<const StateRotation3D&>(as);
			return *this += StateRotation3D(s).invert();
		}
		virtual StateRotation3D operator-(const StateRotation3D& s) const
		{
			StateRotation3D tmp(s);
			return *this + tmp.invert();
		}
		bool operator==(const AbstractState& as) const
		{
			const StateRotation3D& s = static_cast<const StateRotation3D&>(as);
			return State<double,4>::operator==(as) 
				|| State<double,4>::operator==(StateRotation3D(-s[0],-s[1],-s[2],-s[3]));
		}
		virtual StatePosition3D operator*(const StatePosition3D& p) const
		{
			StateRotation3D tmp((*this) + StateRotation3D(0,p[0],p[1],p[2]) 
				+ StateRotation3D(*this).invert());
			StatePosition3D result;
			result[0] = tmp[1];
			result[1] = tmp[2];
			result[1] = tmp[3];
			return result;
		}
		virtual void interpolate(const AbstractState& as, const double t, 
			AbstractState& sNew) const
		{
			const StateRotation3D& s = static_cast<const StateRotation3D&>(as);
			static_cast<StateRotation3D&>(sNew) = *this + exp(tangent(s) * t);
		}
		static const StateRotation3D& min()
		{
			static const StateRotation3D sMin(-1.);
			return sMin;
		}
		static const StateRotation3D max()
		{
			static const StateRotation3D sMax(1.);
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& = min(), 
			const AbstractState& = max()) const
		{
			return std::abs(norm() - 1.) <= epsilon<double>::value;
		}
		virtual StateRotation3D& enforceBounds(const AbstractState& = min(), 
			const AbstractState& = max())
		{
			double nrm = norm();
			if (nrm < epsilon<double>::value)
			{
				(*this)[0] = 1.;
				(*this)[1] = (*this)[2] = (*this)[3] = 0.;
			}
			else
			{
				double scale = 1./nrm;
				(*this)[0] *= scale;
				(*this)[1] *= scale;
				(*this)[2] *= scale;
				(*this)[3] *= scale;
			}
			return *this;
		}
	protected:
		StateRotation3D(const double d) : State<double,4>(d)
		{
		}
		State<double,3> tangent(const StateRotation3D& s) const
		{
			StateRotation3D tmp(StateRotation3D(*this).invert() + s);
			return tmp.log();
		}
		State<double,3> log() const
		{
			State<double,3> result;
			double s = std::min(1., (*this)[0]), sign = (s < 0.) ? -1. : 1.;
			double a;
			if (std::abs(s - 1.) <= epsilon<double>::value ||
				std::abs(s + 1.) <= epsilon<double>::value)
				a = (2 * sign);
			else
			{
				s *= s;
				a = sign * std::acos(2 * s - 1) / sqrt(1 - s);
			}
			result[0] = a * (*this)[1];
			result[1] = a * (*this)[2];
			result[2] = a * (*this)[3];
			return result;
		}
		StateRotation3D exp(const State<double,3>& x) const
		{
			double norm = x.norm(), s = std::cos(.5 * norm), 
				a = (norm <= epsilon<double>::value) ? .5/s : (sqrt(1 - s * s) / norm);
			return StateRotation3D(s, a * x[0], a * x[1], a * x[2]);
		}
	};
	
	}
	
}

#endif
