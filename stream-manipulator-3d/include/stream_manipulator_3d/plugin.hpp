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
#ifndef _PLUGIN_HPP_
#define _PLUGIN_HPP_

#include <stream_manipulator_3d/common_ros.h>
#include <stream_manipulator_3d/common_std.h>
#include <stream_manipulator_3d/common_pcl.h>
#include <stream_manipulator_3d/shared_memory_handler.hpp>

namespace sm3d
{
///Base class  for StreamManipulator Plugins, should  be inherited.
class Plugin
{
    public:
        typedef  boost::shared_ptr<Plugin> Ptr;
        virtual ~Plugin(){}
        Plugin(){}
        //this defines the  plugin namespace, so that  multiple istances can
        //coexists  as long  as they  have  different names  they will  have
        //different parameters
        virtual inline void setNodeHandle(const std::string name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            nh_ = boost::make_shared<ros::NodeHandle>(father_nh, name);
            name_ = nh_->getUnresolvedNamespace();
            priv_nh_ = boost::make_shared<ros::NodeHandle>(priv_nh, name);
        }
        virtual inline std::string getName() const {return name_;}
        virtual void apply(PTC_Ptr input, PTC_Ptr &output)=0;
        //Default plugin has no marker to publish, overload this method to return
        //true if a child has a marker to publish and it needs to be updated
        virtual bool hasMarker() {return false;}
        //To use markers also overload this function which creates the marker
        virtual void createMarker(visualization_msgs::Marker &m){}
        //All plugin must overload an init function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)=0;
    ///////Members
    protected:
        std::string name_;
        ::boost::shared_ptr<ros::NodeHandle> priv_nh_;
        ::boost::shared_ptr<ros::NodeHandle> nh_;
        ::boost::shared_ptr<visualization_msgs::MarkerArray> markers_;
        //Shared Memory Handler
        ShmHandler shm;
};
}
#endif

