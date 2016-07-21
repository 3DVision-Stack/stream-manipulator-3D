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
#ifndef _COMMON_PCL_H_
#define _COMMON_PCL_H_
//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/PointIndices.h>
#include <pcl/pcl_base.h>

namespace sm3d
{
///Convenient PointCloud Typedefs
typedef ::pcl::PointXYZRGB PT; ///< Default point type.
typedef ::pcl::PointCloud<PT> PTC; ///< Default point cloud with default point type.
typedef ::boost::shared_ptr<PTC> PTC_Ptr; ///< Default point cloud with default point type.
typedef ::boost::shared_ptr<const PTC> PTC_ConstPtr; ///< Default point cloud with default point type.

typedef ::pcl::PointXYZ PX; ///< Point type without color
typedef ::pcl::PointCloud<PX> PXC; ///< Point cloud with PX type
typedef ::boost::shared_ptr<PXC> PXC_Ptr; ///< Point cloud with PX type

typedef ::pcl::PointXYZRGBA PTA; ///< Point with color and alpha
typedef ::pcl::PointCloud<PTA> PTAC;
typedef ::boost::shared_ptr<PTAC> PTAC_Ptr;

}//namespace
#endif
