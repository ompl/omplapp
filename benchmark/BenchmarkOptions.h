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

/* Author: Ioan Sucan */

#include <map>
#include <string>
#include <vector>
#include <boost/filesystem/path.hpp>

struct BenchmarkOptions
{
    BenchmarkOptions(void)
    {
    }

    BenchmarkOptions(const char *filename)
    {
        readOptions(filename);
    }

    using PlannerOpt = std::map<std::string, std::string>; // options specific to the planner
    using ContextOpt = std::map<std::string, std::string>; // other options (not specific to the planner)

    // all the options for a benchmark execution
    struct AllOptions
    {
        ContextOpt c;
        PlannerOpt p;
    };

    std::map<std::string, std::string>              declared_options_;
    std::map<std::string, std::vector<AllOptions> > planners_;

    // the path where the input .cfg file is located
    boost::filesystem::path                         path_;

    // the file to which benchmark results should be written
    boost::filesystem::path                         outfile_;

    bool readOptions(const char *filename);
    bool isSE2Problem(void) const;
    bool isSE3Problem(void) const;
};
