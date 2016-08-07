/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan, Mark Moll */

#include "BenchmarkTypes.h"
#include <iostream>

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage:\n\t " << argv[0] << " problem.cfg" << std::endl;
        return 1;
    }

    BenchmarkOptions bo;
    if (bo.readOptions(argv[1]))
    {
        std::shared_ptr<CFGBenchmark> b;
        auto controlType = bo.declared_options_.find("problem.control");
        if (bo.isSE2Problem())
        {
            if (controlType == bo.declared_options_.end())
                b = std::make_shared<SE2Benchmark>(bo);
            else if (controlType->second == "kinematic_car")
                b = std::make_shared<KinematicCarBenchmark>(bo);
            else if (controlType->second == "dynamic_car")
                b = std::make_shared<DynamicCarBenchmark>(bo);
            else
                b = std::make_shared<SE2Benchmark>(bo);
        }
        else if (bo.isSE3Problem())
        {
            if (controlType == bo.declared_options_.end())
                b = std::make_shared<SE3Benchmark>(bo);
            else if (controlType->second == "blimp")
                b = std::make_shared<BlimpBenchmark>(bo);
            else if (controlType->second == "quadrotor")
                b = std::make_shared<QuadrotorBenchmark>(bo);
            else
                b = std::make_shared<SE3Benchmark>(bo);
        }

        if (b)
        {
            b->setup();
            b->runBenchmark();
        }
        else
            std::cerr << "No known benchmark" << std::endl;
    }
    else
        std::cerr << "Unable to load options" << std::endl;

    return 0;
}
