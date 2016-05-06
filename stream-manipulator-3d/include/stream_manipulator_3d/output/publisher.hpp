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
#ifndef _OUTPUT_PUBLISHER_HPP_
#define _OUTPUT_PUBLISHER_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/output/config/publisher_config.hpp>
#include <stream_manipulator_3d/common_pcl.h>

namespace sm3d
{
namespace output
{
///Publisher of streams.
class Publisher : public sm3d::Plugin
{
    public:
        virtual ~Publisher()
        {
            clean();
        }
        Publisher() : Plugin()
        {
        }
        ///initialize function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            setNodeHandle(name, father_nh, priv_nh);
            //Then do our specific configuration
            //Create config in shared_memory
            config = shm.segment.construct<PublisherConfig>((name_+"Config").c_str())(shm.char_alloc);

            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);

            //////////////Init Parameters///////////////////////////////////////
            if (nh_->hasParam("output_topic")){
                nh_->getParam("output_topic", topic);
                config->output_topic = topic.c_str();
            }
            else{
                nh_->setParam("output_topic", config->output_topic.c_str());
                topic = config->output_topic.c_str();
            }
            //Disabled flag, tells if the filter should be applied or not
            //Starts as disabled
            if (nh_->hasParam("disabled"))
                nh_->getParam("disabled", config->disabled);
            else
                nh_->setParam("disabled", config->disabled);
            //Advertise
            pub = nh_->advertise<PTC>(topic,0);
            ROS_INFO("[%s::%s] Initialization complete",name_.c_str(),__func__);
        }
        /// apply() implementation
        virtual void apply(PTC::Ptr input, PTC::Ptr &output)
        {
            if(!input){
                ROS_WARN_THROTTLE(30,"[%s::%s]\tNo input cloud, aborting...",name_.c_str(),__func__);
                return;
            }
            if(input->empty()){
                ROS_WARN_THROTTLE(30,"[%s::%s]\tEmpty input cloud, aborting...",name_.c_str(),__func__);
                return;
            }
            if(!output)
                output=boost::make_shared<PTC>();
            //Propagate stream
            output = input;
            output->header.frame_id = input->header.frame_id;
            //Lock config mutex
            ShmHandler::Lock  lock(config->mtx);
            if (config->disabled){
                //disabled, just copy input into output
                return;
            }
            if (topic.compare(config->output_topic.c_str()) != 0){
                //topic changed
                pub.shutdown();
                topic = config->output_topic.c_str();
                //Re-advertise
                pub = nh_->advertise<PTC>(topic,0);
                ROS_INFO("[%s::%s] Advertising to %s",name_.c_str(),__func__,topic.c_str());
            }
            pub.publish(*output);
        }
    protected:
    ///////Members
    //  Configuration in shared memory
        PublisherConfig *config;
        ros::Publisher pub;
        std::string topic;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("disabled");
            nh_->deleteParam("output_topic");
            shm.segment.destroy<PublisherConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }
};
}//ns
}//ns output
#endif

