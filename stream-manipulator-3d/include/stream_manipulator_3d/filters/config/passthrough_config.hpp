// Software License Agreement (BSD License)
//
//   Stream Manipulator 3d - https://github.com/3DVision-Stack/stream-manipulator-3D
//   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
//   All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder(s) nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#ifndef _FILTERS_PASSTHROUGH_CONFIG_HPP_
#define _FILTERS_PASSTHROUGH_CONFIG_HPP_

#include <stream_manipulator_3d/shared_memory_handler.hpp>

namespace sm3d
{
namespace filters
{
///PassThrough Filter Configuration, this lives in shared memory
struct PassThroughConfig
{
    boost::interprocess::interprocess_mutex mtx;
    bool organized,negative,disabled;
    //Since point type is hardcoded for now, it's easier this way.
    //TODO review this when point type will be runtime choosable
    int field; //enumerator like to list fields:  x,y,z,rgb
    double l_min, l_max;
    double new_value;
    PassThroughConfig(): organized(false), negative(false), disabled(true), field(0),
    l_min(-0.5),l_max(0.5), new_value(std::numeric_limits<double>::quiet_NaN())
    {}
};
}//ns
}//ns filters
#endif

