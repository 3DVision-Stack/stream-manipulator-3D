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
#ifndef _FILTERS_CROPBOX_CONFIG_HPP_
#define _FILTERS_CROPBOX_CONFIG_HPP_

#include <stream_manipulator_3d/shared_memory_handler.hpp>

namespace sm3d
{
namespace filters
{
///CropBox Filter Configuration, this lives in shared memory
struct CropBoxConfig
{
    boost::interprocess::interprocess_mutex mtx;
    bool organized,negative,pub_marker,disabled;
    double lim_x1,lim_x2,lim_y1,lim_y2,lim_z1,lim_z2;
    double qw,qx,qy,qz,tx,ty,tz;
    double color_r,color_g,color_b;
    bool lim_changed, trans_changed;
    CropBoxConfig(): organized(false), negative(false), pub_marker(true), disabled(true),
    lim_x1(-0.5),lim_x2(0.5),lim_y1(-0.5),lim_y2(0.5),lim_z1(-0.5), lim_z2(1.0),
    qw(1.0), qx(0.0), qy(0.0), qz(0.0), tx(0.0), ty(0.0), tz(0.0),
    color_r(1.0),color_g(1.0),color_b(1.0), lim_changed(true), trans_changed(true)
    {}
};
}//ns
}//ns filters
#endif

