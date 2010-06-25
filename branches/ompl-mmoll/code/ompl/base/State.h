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

#ifndef OMPL_BASE_STATE_
#define OMPL_BASE_STATE_

#include <iostream>
#include <iterator>
#include <limits>
#include <functional>
#include <numeric>
#include <cstdlib>
#include <cmath>
#include <boost/array.hpp>
#include "ompl/base/General.h"

namespace ompl
{
	namespace base
	{

	/// Negligible difference between state components
	template<class T>
	struct epsilon
	{
		static const T value = 1e-6;
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
		virtual AbstractState& invert() = 0;
		virtual AbstractState& operator+=(const AbstractState&) = 0;
		virtual AbstractState& operator-=(const AbstractState&) = 0;
		virtual bool operator==(const AbstractState&) const = 0;
		virtual void interpolate(const AbstractState&, const double, AbstractState&) const = 0;
		virtual bool satisfiesBounds(const AbstractState&, const AbstractState&) const = 0;
		virtual AbstractState& enforceBounds(const AbstractState&, const AbstractState&) = 0;
	};
	
	/// Definition of a state
	template<class T, std::size_t N>
	class State : public AbstractState, public boost::array<T,N>
	{
	public:
		State() : boost::array<T,N>() {}
		State(const T t)
		{
			assign(t);
		}
		State(const T x, const T y)
		{
			assert(N>1);
			(*this)[0] = x;
			(*this)[1] = y;
		}
		State(const T x, const T y, const T z)
		{
			assert(N>2);
			(*this)[0] = x;
			(*this)[1] = y;
			(*this)[2] = z;
		}
		State* clone()
		{
			return new State(*this);
		}
		virtual double norm_sq() const
		{
			double sum=0, x;
			for (std::size_t i=0; i<N; ++i)
			{
				x = (*this)[i];
				sum += x*x;
			}
			return sum;
		}
		virtual double norm() const
		{
			return std::sqrt(norm_sq());
		
		}
		virtual double distance(const AbstractState& s) const
		{
			return (*this - static_cast<const State&>(s)).norm();
		}
		virtual State& invert()
		{
			std::transform(this->begin(), this->end(), this->begin(),
				std::negate<T>());
			return *this;
		}
		virtual State& operator+=(const AbstractState& s)
		{
			std::transform(this->begin(), this->end(), static_cast<const State&>(s).begin(),
				this->begin(), std::plus<T>());
			return *this;
		}
		virtual State operator+(const State& s) const
		{
			return State(*this) += s;
		}
		virtual State& operator-=(const AbstractState& s)
		{
			std::transform(this->begin(), this->end(), static_cast<const State&>(s).begin(),
				this->begin(), std::minus<T>());
			return *this;
		}
		virtual State operator-(const State& s) const
		{
			return State(*this) -= s;
		}
		virtual State operator*(const T t) const
		{
			State result;
			for (std::size_t i=0; i<N; ++i) 
				result[i] = t * (*this)[i];
			return result;
		}
		virtual bool operator==(const AbstractState& as) const
		{
			const State& s = static_cast<const State&>(as);
			for(std::size_t i=0; i<N; ++i)
				if (std::abs((*this)[i]-s[i]) > epsilon<T>::value)
					return false;
			return true;
		}
		virtual void interpolate(const AbstractState& s, const double t, AbstractState& sNew) const
		{
			static_cast<State&>(sNew) = *this + (static_cast<const State&>(s)-*this)*t;
		}
		virtual T dot(const State& s) const
		{
			return std::inner_product(this->begin(),this->end(),s.begin(),0);
		}
		static const State& min()
		{
			static const State sMin(-std::numeric_limits<T>::max());
			return sMin;
		}
		static const State& max()
		{
			static const State sMax(std::numeric_limits<T>::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo = min(), 
			const AbstractState& ahi = max()) const
		{
			const State& lo = static_cast<const State&>(alo);
			const State& hi = static_cast<const State&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if ((*this)[i] < lo[i] || (*this)[i] > hi[i])
					return false;
			return true;
		}
		virtual State& enforceBounds(const AbstractState& alo = min(), 
			const AbstractState& ahi = max())
		{
			const State& lo = static_cast<const State&>(alo);
			const State& hi = static_cast<const State&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if ((*this)[i] < lo[i])
					(*this)[i] = lo[i];
				else if ((*this)[i] > hi[i])
					(*this)[i] = hi[i];
			return *this;
		}
	};
	typedef State<double, 1> StatePosition1D;
	typedef State<double, 2> StatePosition2D;
	typedef State<double, 3> StatePosition3D;
	 
	/// Definition of a composite state
	template<std::size_t N>
	class StateComposite : public AbstractState, public boost::array<AbstractState*,N>
	{
	public:
		StateComposite() : boost::array<AbstractState*,N>() 
		{
			this->assign(NULL);
		}
		StateComposite(const StateComposite& s)
		{
			for (std::size_t i=0; i<N; ++i)
				(*this)[i] = s[i]->clone();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			double d=0.;
			for (std::size_t i=0; i<N; ++i)
				d += (*this)[i]->distance(*s[i]);
			return d;
		}
		virtual StateComposite& invert()
		{
			for (std::size_t i=0; i<N; ++i)
				(*this)[i]->invert();
			return *this;
		}
		virtual StateComposite& operator+=(const AbstractState& as)
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for (std::size_t i=0; i<N; ++i)
				*(*this)[i] += *s[i];
			return *this;
		}
		virtual StateComposite operator+(const StateComposite& s) const
		{
			return StateComposite(*this) += s;
		}
		virtual StateComposite& operator-=(const AbstractState& as)
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for (std::size_t i=0; i<N; ++i)
				*(*this)[i] -= *s[i];
			return *this;
		}
		virtual StateComposite operator-(const StateComposite& s) const
		{
			return StateComposite(*this) -= s;
		}
		virtual bool operator==(const AbstractState& as) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			for(std::size_t i=0; i<N; ++i)
				if (!((*this)[i] == *s[i]))
					return false;
			return true;
		}
		virtual void interpolate(const AbstractState& as, const double t, AbstractState& asNew) const
		{
			const StateComposite& s = static_cast<const StateComposite&>(as);
			StateComposite& sNew = static_cast<StateComposite&>(asNew);
			for(std::size_t i=0; i<N; ++i)
				(*this)[i]->interpolate(s[i], t, *sNew[i]);
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StateComposite& lo = static_cast<const StateComposite&>(alo);
			const StateComposite& hi = static_cast<const StateComposite&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				if (!(*this)[i]->satisfiesBounds(*lo[i], *hi[i]))
					return false;
			return true;
		}
		virtual StateComposite& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StateComposite& lo = static_cast<const StateComposite&>(alo);
			const StateComposite& hi = static_cast<const StateComposite&>(ahi);
			for (std::size_t i=0; i<N; ++i)
				(*this)[i]->enforceBounds(*lo[i], *hi[i]);
			return *this;
		}
	};

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
	
	template <class Rotation, class Translation>
	class StatePose : public AbstractState, public std::pair<Rotation, Translation>
	{
	public:
		StatePose()
		{
		}
		StatePose(const Rotation& rot, const Translation& trans)
			: std::pair<Rotation,Translation>(rot, trans)
		{
		}
		StatePose* clone()
		{
			return new StatePose(*this);
		}
		static std::size_t size()
		{
			return Rotation::size() + Translation::size();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return this->first.distance(s.first) + this->second.distance(s.second);
		}
		virtual StatePose& invert()
		{
			this->first.invert();
			this->second = this->first * this->second.invert();
			return *this;
		}
		virtual StatePose& operator+=(const AbstractState& as)
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			this->second += this->first * s.second;
			this->first += s.first;
			return *this;
		}
		virtual StatePose operator+(const StatePose& s) const
		{
			return StatePose(*this) += s;
		}
		virtual StatePose operator*(const StatePose& s) const
		{
			return StatePose(*this) += s;
		}
		virtual StatePose& operator-=(const AbstractState& as)
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return StatePose(*this) += StatePose(s).invert();
		}
		virtual StatePose operator-(const StatePose& s) const
		{
			return StatePose(*this) += StatePose(s).invert();
		}
		virtual StatePose operator*(const double t) const
		{
			return StatePose(this->first, this->second * t);
		}
		virtual Translation operator*(const Translation& t) const
		{
			return this->second + this->first * t;
		}
		bool operator==(const AbstractState& as) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			return this->first == s.first && this->second == s.second;
		}
		virtual void interpolate(const AbstractState& as, const double t, 
			AbstractState& sNew) const
		{
			const StatePose& s = static_cast<const StatePose&>(as);
			Rotation rot;
			Translation trans;
			this->first.interpolate(s.first, t, rot);
			this->second.interpolate(s.second, t, trans);
			static_cast<StatePose&>(sNew) = StatePose(rot, trans);
		}
		static const StatePose& min()
		{
			static const StatePose sMin(Rotation::min(),Translation::min());
			return sMin;
		}
		static const StatePose& max()
		{
			static const StatePose sMax(Rotation::max(),Translation::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StatePose& lo = static_cast<const StatePose&>(alo);
			const StatePose& hi = static_cast<const StatePose&>(ahi);
			return this->first.satisfiesBounds(lo.first, hi.first)
				&& this->second.satisfiesBounds(lo.second, hi.second);
		}
		virtual StatePose& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StatePose& lo = static_cast<const StatePose&>(alo);
			const StatePose& hi = static_cast<const StatePose&>(ahi);
			this->first.enforceBounds(lo.first, hi.first);
			this->second.enforceBounds(lo.second, hi.second);
			return *this;
		}
		
	};
	typedef StatePose<StateRotation2D, StatePosition2D> StatePose2D;
	typedef StatePose<StateRotation3D, StatePosition3D> StatePose3D;
	
	
	/// Definition of a hybrid state
	template<class Discrete, class Continuous>
	class StateHybrid : public AbstractState, public std::pair<Discrete, Continuous>
	{
	public:
		StateHybrid()
		{
		}
		StateHybrid(const Discrete& mode, const Continuous& state)
			: std::pair<Discrete,Continuous>(mode, state)
		{
		}
		StateHybrid* clone()
		{
			return new StateHybrid(*this);
		}
		static std::size_t size(bool continousPartOnly=true)
		{
			return continousPartOnly ? Continuous::size() 
				: Discrete::size() + Continuous::size();
		}
		int& mode(unsigned int i)
		{
			return this->first[i];
		}
		double& operator[](unsigned int i)
		{
			return this->second[i];
		}
		static std::size_t size()
		{
			return Discrete::size() + Continuous::size();
		}
		virtual double distance(const AbstractState& as) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return this->first.distance(s.first) + this->second.distance(s.second);
		}
		virtual StateHybrid& invert()
		{
			this->second.invert();
			return *this;
		}
		virtual StateHybrid& operator+=(const AbstractState& as)
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			this->second += s.second;
			assert(this->first == s.first);
			return *this;
		}
		virtual StateHybrid operator+(const StateHybrid& s) const
		{
			return StateHybrid(*this) += s;
		}
		virtual StateHybrid operator*(const StateHybrid& s) const
		{
			return StateHybrid(*this) += s;
		}
		virtual StateHybrid& operator-=(const AbstractState& as)
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return StateHybrid(*this) += StateHybrid(s).invert();
		}
		virtual StateHybrid operator-(const StateHybrid& s) const
		{
			return StateHybrid(*this) += StateHybrid(s).invert();
		}
		virtual StateHybrid operator*(const double t) const
		{
			return StateHybrid(this->first, this->second * t);
		}
		bool operator==(const AbstractState& as) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			return this->first == s.first && this->second == s.second;
		}
		virtual void interpolate(const AbstractState& as, const double t, 
			AbstractState& sNew) const
		{
			const StateHybrid& s = static_cast<const StateHybrid&>(as);
			Continuous c;
			assert(this->first == s.first);
			this->second.interpolate(s.second, t, c);
			static_cast<StateHybrid&>(sNew) = StateHybrid(this->first, c);
		}
		static const StateHybrid min()
		{
			static const StateHybrid sMin(Discrete::min(),Continuous::min());
			return sMin;
		}
		static const StateHybrid max() 
		{
			static const StateHybrid sMax(Discrete::max(),Continuous::max());
			return sMax;
		}
		virtual bool satisfiesBounds(const AbstractState& alo, const AbstractState& ahi) const
		{
			const StateHybrid& lo = static_cast<const StateHybrid&>(alo);
			const StateHybrid& hi = static_cast<const StateHybrid&>(ahi);
			return this->first.satisfiesBounds(lo.first, hi.first)
				&& this->second.satisfiesBounds(lo.second, hi.second);
		}
		virtual StateHybrid& enforceBounds(const AbstractState& alo, const AbstractState& ahi)
		{
			const StateHybrid& lo = static_cast<const StateHybrid&>(alo);
			const StateHybrid& hi = static_cast<const StateHybrid&>(ahi);
			this->first.enforceBounds(lo.first, hi.first);
			this->second.enforceBounds(lo.second, hi.second);
			return *this;
		}
	};
	
	
	template<class T, std::size_t N>
	std::ostream& operator<<(std::ostream& out, const State<T,N>& s)
	{
		std::ostream_iterator<double> out_it (out," ");
		std::copy(s.begin(), s.end(), out_it );
		return out;
	}
	
	template<class T1, class T2>
	std::ostream& operator<<(std::ostream& out, const std::pair<T1, T2>& s)
	{
		return out << s.first << "| " << s.second;
	}
	
	}

}

#endif
