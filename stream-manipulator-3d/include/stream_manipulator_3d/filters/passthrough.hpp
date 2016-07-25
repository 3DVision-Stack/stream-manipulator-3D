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
#ifndef _FILTERS_PASSTHROUGH_HPP_
#define _FILTERS_PASSTHROUGH_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/filters/config/passthrough_config.hpp>
#include <pcl/filters/passthrough.h>

namespace sm3d
{
namespace filters
{
///PassThrough Filter, wrapping pcl one.
class PassThrough : public sm3d::Plugin
{
    public:
        virtual ~PassThrough()
        {
            clean();
        }
        PassThrough() : Plugin(), field(0)
        {
        }
        ///initialize function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            setNodeHandle(name, father_nh, priv_nh);
            //Then do our specific configuration
            //Create config in shared_memory
            config = shm.segment.construct<PassThroughConfig>((name_+"Config").c_str())();

            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);

            //////////////Init Parameters///////////////////////////////////////
            //Organized Flag, keeps point cloud organized when possible
            if (nh_->hasParam("organized"))
                nh_->getParam("organized", config->organized);
            else
                nh_->setParam("organized", config->organized);
            //Negative Flag, tells if the filter needs to remove points inside limits
            if (nh_->hasParam("negative"))
                nh_->getParam("negative", config->negative);
            else
                nh_->setParam("negative", config->negative);
            //Disabled flag, tells if the filter should be applied or not
            //Starts as disabled
            if (nh_->hasParam("disabled"))
                nh_->getParam("disabled", config->disabled);
            else
                nh_->setParam("disabled", config->disabled);
            //Filter Field enumerator
            //0 -> x
            //1 -> y
            //2 -> z
            //3 -> rgb
            if (nh_->hasParam("filter_field")){
                nh_->getParam("filter_field",field);
                if (field < 0 || field >3)
                    ROS_WARN("[%s::%s]\tInconsistent filter_field, resetting to default...",name_.c_str(),__func__);
                else
                    config->field = field;
            }
            else
                nh_->setParam("filter_field",config->field);
            field = config->field;
            selectField(field);
            //Minimum value of filter, for the chosen field
            if (nh_->hasParam("filter_lim_min"))
                nh_->getParam("filter_lim_min",config->l_min);
            else
                nh_->setParam("filter_lim_min",config->l_min);
            //Maximum value of filter, for the chosen field
            if (nh_->hasParam("filter_lim_max"))
                nh_->getParam("filter_lim_max",config->l_max);
            else
                nh_->setParam("filter_lim_max",config->l_max);
            //If keep organized, put filtered out points to this value instead of NaN
            if (nh_->hasParam("filter_value"))
                nh_->getParam("filter_value",config->new_value);
            else
                nh_->setParam("filter_value",config->new_value);
            ROS_INFO("[%s::%s] Initialization complete",name_.c_str(),__func__);
        }
        /// apply() implementation
        virtual void apply(PTC_Ptr input, PTC_Ptr &output)
        {
            if(!input){
                ROS_WARN_THROTTLE(30,"[%s::%s]\tNo input cloud, aborting...",name_.c_str(),__func__);
                return;
            }
            if(input->empty()){
                ROS_WARN_THROTTLE(30,"[%s::%s]\tEmpty input cloud, aborting...",name_.c_str(),__func__);
                return;
            }
            //Lock cropbox config mutex
            ShmHandler::Lock  lock(config->mtx);
            if (config->disabled){
                //Filter is disabled, just copy input into output
                output = input;
                return;
            }
            pt.setKeepOrganized(config->organized);
            pt.setNegative(config->negative);
            if (field != config->field){
                field = config->field;
                selectField(field);
            }
            pt.setFilterLimits(config->l_min, config->l_max);
            pt.setUserFilterValue(config->new_value);
            pt.setInputCloud(input);
            pt.filter (*output);
            output->header.frame_id = input->header.frame_id;
        }
    protected:
    ///////Members
    //  Configuration in shared memory
        PassThroughConfig *config;
        //Pcl cropbox obj
        ::pcl::PassThrough<PT> pt;
        //field type
        int field;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("organized");
            nh_->deleteParam("negative");
            nh_->deleteParam("disabled");
            nh_->deleteParam("filter_field");
            nh_->deleteParam("filter_lim_max");
            nh_->deleteParam("filter_lim_min");
            shm.segment.destroy<PassThroughConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }

        void selectField(int f)
        {
            if (f <0 || f >3)
                f = 0;
            switch (f)
            {
                case 0:
                    pt.setFilterFieldName("x");
                    break;
                case 1:
                    pt.setFilterFieldName("y");
                    break;
                case 2:
                    pt.setFilterFieldName("z");
                    break;
                case 3:
                    pt.setFilterFieldName("rgb");
                    break;
            }
        }
};
}//ns
}//ns filters
#endif

