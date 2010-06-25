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

/* \author Mark Moll */

#include <gtest/gtest.h>
#include "ompl/base/State.h"

using namespace ompl::base;

TEST(State, SimpleDouble)
{
	State<double,3> s(0.), s2(1e-7), s3(1.);
	EXPECT_EQ(s.norm_sq(), 0.);
	EXPECT_EQ(s.norm(), 0.);
	EXPECT_EQ(s, s2);
	EXPECT_NE(s, s3);
	EXPECT_EQ(s.distance(s3), sqrt(3.));
	EXPECT_EQ(s3.norm_sq(), 3.);
	EXPECT_EQ(s3.norm(), sqrt(3.));
	s2 = s + s3;
	EXPECT_EQ(s3, s2);
	s2 = s2 - s3;
	EXPECT_EQ(s2, s);
	EXPECT_EQ(s.dot(s), 0);
	EXPECT_EQ(s3.dot(s3), 3);
	EXPECT_EQ(s3*1., s3);
	EXPECT_EQ(s*10, s);
	s.interpolate(s3, .5, s2);
	EXPECT_EQ(s3*.5, s2);
	EXPECT_TRUE(s2.satisfiesBounds());
	EXPECT_TRUE(s3.satisfiesBounds(s3,s3));
	EXPECT_FALSE(s2.satisfiesBounds(s3,s3));
	s2.enforceBounds(s3,s3);
	EXPECT_EQ(s2,s3);
	
	StatePosition3D p(1.,2.,3.), q = p, zero(0.);
	q.invert();
	EXPECT_EQ(p + q, zero);
}

TEST(State, SimpleInt)
{
	State<int,2> s(0), s2(0), s3;
	s3[0] = 3; 
	s3[1] = 4;
	EXPECT_EQ(s.norm_sq(), 0.);
	EXPECT_EQ(s.norm(), 0.);
	EXPECT_EQ(s, s2);
	EXPECT_NE(s, s3);
	EXPECT_EQ(s.distance(s3), 5);
	EXPECT_EQ(s3.norm_sq(), 25);
	EXPECT_EQ(s3.norm(), 5);
	s2 = s + s3;
	EXPECT_EQ(s3, s2);
	s2 = s2 - s3;
	EXPECT_EQ(s2, s);
	EXPECT_EQ(s.dot(s), 0);
	EXPECT_EQ(s3.dot(s3), 25);
	EXPECT_EQ(s3*1, s3);
	EXPECT_EQ(s*10, s);
	s.interpolate(s3, 2, s2);
	EXPECT_EQ(s3*2, s2);
	EXPECT_TRUE(s2.satisfiesBounds());
	EXPECT_TRUE(s3.satisfiesBounds(s3, s3));
	EXPECT_FALSE(s2.satisfiesBounds(s3, s3));
	s2.enforceBounds(s3, s3);
	EXPECT_EQ(s2,s3);
}

TEST(State, Rotation2D)
{
	StateRotation2D s0, s1(M_PI_4), s2(M_PI_2), s3(1.9999999*M_PI), s;
	
	EXPECT_NE(s0, s1);
	EXPECT_NE(s0, s2);
	EXPECT_EQ(s0, s3);
	s = StateRotation2D(s3).invert();
	EXPECT_EQ(s0, s);
	EXPECT_EQ(s3, s);
	s = s1 + s1;
	EXPECT_EQ(s2, s);
	EXPECT_TRUE(s0.satisfiesBounds());
	EXPECT_TRUE(s1.satisfiesBounds());
	EXPECT_TRUE(s2.satisfiesBounds());	
	EXPECT_TRUE(s3.satisfiesBounds());	
	s = s3 + s3;
	EXPECT_FALSE(s.satisfiesBounds());
	EXPECT_EQ(s, s0);
	s.enforceBounds();
	EXPECT_TRUE(s.satisfiesBounds());
	s = s2 - s1 - s1 - s3;
	EXPECT_EQ(s, s0);
	EXPECT_EQ(StateRotation2D::min(), 0.);
	EXPECT_EQ(StateRotation2D::max(), 2*M_PI);
	s = s3;
	s.invert();
	EXPECT_EQ(s + s3, s0);
	EXPECT_EQ(s * s3, s0);
	s0.interpolate(s1, 0., s);
	EXPECT_EQ(s0, s);
	s0.interpolate(s1, 1., s);
	EXPECT_EQ(s1, s);
	s0.interpolate(s1, 2., s);
	EXPECT_EQ(s2, s);
	s0.interpolate(s1, 8., s);
	EXPECT_EQ(s0, s);
}

TEST(State, Rotation3D)
{
	StateRotation3D s0, s1(1.,1.,1.,1.), s2(1.,2.,3.,4.), s;	
	EXPECT_NE(s0, s1);
	EXPECT_NE(s0, s2);
	EXPECT_FALSE(s1.satisfiesBounds());
	s1.enforceBounds();
	EXPECT_TRUE(s1.satisfiesBounds());
	EXPECT_FALSE(s2.satisfiesBounds());
	s2.enforceBounds();
	EXPECT_TRUE(s2.satisfiesBounds());
	s = (s1 * s2).invert();
	EXPECT_EQ(s * s1 * s2, s0);
	EXPECT_EQ(s + s1 + s2, s0);
	EXPECT_EQ(s1 * s2 * s, s0);
	EXPECT_EQ(s1 + s2 + s, s0);
	EXPECT_EQ(s0.distance(s0), 0.);
	EXPECT_EQ(s1.distance(s1), 0.);
	EXPECT_EQ(s2.distance(s2), 0.);
	s0.interpolate(s1, 0., s);
	EXPECT_EQ(s0, s);
	s0.interpolate(s1, 1., s);
	EXPECT_EQ(s1, s);
	s1.interpolate(s2, 1., s);
	EXPECT_EQ(s2, s);
}

TEST(State, Pose2D)
{
	StatePosition2D p0(0., 0.), p1(0., 1.), p2(-1., 0.), p;
	StateRotation2D R0, R1(M_PI_4), R;
	StatePose2D pose0(R0, p0), pose1(R1, p1), pose;
	
	EXPECT_EQ(R0 * p1, p1);
	EXPECT_EQ(R1 * R1 * p1, p2);
	R = StateRotation2D(2*M_PI);
	EXPECT_EQ(R * p1, p1);
	pose = StatePose2D(pose1).invert();
	EXPECT_EQ(pose0, pose * pose1);
	EXPECT_EQ(pose0, pose1 * pose);
	
	pose1 = StatePose2D(StateRotation2D(M_PI_2),StatePosition2D(1., 0.));
	pose = pose1 * pose0;
	pose = pose1 * pose;
	pose = pose1 * pose;
	pose = pose1 * pose;
	EXPECT_EQ(pose0, pose);
}

TEST(State, Pose3D)
{
}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
