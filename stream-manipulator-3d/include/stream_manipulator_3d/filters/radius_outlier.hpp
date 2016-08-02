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
#ifndef _FILTERS_RADIUS_OUTLIER_HPP_
#define _FILTERS_RADIUS_OUTLIER_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/filters/config/radius_outlier_config.hpp>
#include <pcl/filters/radius_outlier_removal.h>

namespace sm3d
{
namespace filters
{
///RadiusOutlier Filter, wrapping pcl one.
class RadiusOutlier : public sm3d::Plugin
{
    public:
        virtual ~RadiusOutlier()
        {
            clean();
        }
        RadiusOutlier() : Plugin()
        {
        }
        ///initialize function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            setNodeHandle(name, father_nh, priv_nh);
            //Then do our specific configuration
            //Create config in shared_memory
            config = shm.segment.construct<RadiusOutlierConfig>((name_+"Config").c_str())();
            reconfigFromRosParams();
            ROS_INFO("[%s::%s] Initialization complete",name_.c_str(),__func__);
        }
        virtual void reconfigFromRosParams()
        {
            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);

            //Disabled flag, tells if the filter should be applied or not
            //Starts as disabled
            if (nh_->hasParam("disabled"))
                nh_->getParam("disabled", config->disabled);
            else
                nh_->setParam("disabled", config->disabled);
            //Organized flag
            if (nh_->hasParam("organized"))
                nh_->getParam("organized", config->organized);
            else
                nh_->setParam("organized", config->organized);
            //Negative flag
            if (nh_->hasParam("negative"))
                nh_->getParam("negative",config->negative);
            else
                nh_->setParam("negative",config->negative);
            //Search radius for each point
            if (nh_->hasParam("radius_search"))
                nh_->getParam("radius_search",config->radius);
            else
                nh_->setParam("radius_search",config->radius);
            //Minimum number of neighbors to found to classify a point as an inlier
            if (nh_->hasParam("k_neighbors_threshold"))
                nh_->getParam("k_neighbors_threshold",config->k_thresh);
            else
                nh_->setParam("k_neighbors_threshold",config->k_thresh);
        }
        virtual void saveConfigToRosParams()
        {
            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);
            nh_->setParam("disabled", config->disabled);
            nh_->setParam("organized", config->organized);
            nh_->setParam("negative",config->negative);
            nh_->setParam("radius_search",config->radius);
            nh_->setParam("k_neighbors_threshold",config->k_thresh);
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
            //Lock config mutex
            ShmHandler::Lock  lock(config->mtx);
            if (config->disabled){
                //Filter is disabled, just copy input into output
                output = input;
                return;
            }
            //Pcl RadiusOutlier obj
            ::pcl::RadiusOutlierRemoval<PT> ror;
            ror.setMinNeighborsInRadius(config->k_thresh);
            ror.setRadiusSearch(config->radius);
            ror.setKeepOrganized(config->organized);
            ror.setNegative(config->negative);
            ror.setInputCloud(input);
            ror.filter(*output);
            output->header.frame_id = input->header.frame_id;
        }
    protected:
    ///////Members
    //  Configuration in shared memory
        RadiusOutlierConfig *config;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("disabled");
            nh_->deleteParam("negative");
            nh_->deleteParam("organized");
            nh_->deleteParam("radius_search");
            nh_->deleteParam("k_neighbors_threshold");
            shm.segment.destroy<RadiusOutlierConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }
};
}//ns
}//ns filters
#endif

