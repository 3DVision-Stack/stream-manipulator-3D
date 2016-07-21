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
#ifndef _FILTERS_VOXEL_GRID_HPP_
#define _FILTERS_VOXEL_GRID_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/filters/config/voxel_grid_config.hpp>
#include <pcl/filters/voxel_grid.h>
/* #include <pcl/filters/impl/voxel_grid.hpp> */
/* #include <pcl/impl/pcl_base.hpp> */
/* #include <pcl/impl/point_types.hpp> */
/* #include <pcl/filters/impl/filter.hpp> */

namespace sm3d
{
namespace filters
{
///VoxelGrid Filter, wrapping pcl one.
class VoxelGrid : public sm3d::Plugin
{
    public:
        virtual ~VoxelGrid()
        {
            clean();
        }
        VoxelGrid() : Plugin()
        {
        }
        ///initialize function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            setNodeHandle(name, father_nh, priv_nh);
            //Then do our specific configuration
            //Create config in shared_memory
            config = shm.segment.construct<VoxelGridConfig>((name_+"Config").c_str())();

            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);

            //Disabled flag, tells if the filter should be applied or not
            //Starts as disabled
            if (nh_->hasParam("disabled"))
                nh_->getParam("disabled", config->disabled);
            else
                nh_->setParam("disabled", config->disabled);
            //Downsample_all_data flag, tells if the filter should downsample all 
            //fields of the pointcloud or just the XYZ
            if (nh_->hasParam("downsample_all_data"))
                nh_->getParam("downsample_all_data", config->downsample_all_data);
            else
                nh_->setParam("downsample_all_data", config->downsample_all_data);
            //Leaf sizes X
            if (nh_->hasParam("leaf_x"))
                nh_->getParam("leaf_x",config->leaf_x);
            else
                nh_->setParam("leaf_x",config->leaf_x);
            //Leaf sizes Y
            if (nh_->hasParam("leaf_y"))
                nh_->getParam("leaf_y",config->leaf_y);
            else
                nh_->setParam("leaf_y",config->leaf_y);
            //Leaf sizes Z
            if (nh_->hasParam("leaf_z"))
                nh_->getParam("leaf_z",config->leaf_z);
            else
                nh_->setParam("leaf_z",config->leaf_z);
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
            //Lock config mutex
            ShmHandler::Lock  lock(config->mtx);
            if (config->disabled){
                //Filter is disabled, just copy input into output
                output = input;
                return;
            }
            //Pcl VoxelGrid obj
            ::pcl::VoxelGrid<PT> vg;
            vg.setLeafSize(config->leaf_x, config->leaf_y, config->leaf_z);
            vg.setDownsampleAllData(config->downsample_all_data);
            vg.setInputCloud(input);
            vg.filter(*output);
            output->header.frame_id = input->header.frame_id;
        }
    protected:
    ///////Members
    //  Configuration in shared memory
        VoxelGridConfig *config;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("disabled");
            nh_->deleteParam("downsample_all_data");
            nh_->deleteParam("leaf_x");
            nh_->deleteParam("leaf_y");
            nh_->deleteParam("leaf_z");
            shm.segment.destroy<VoxelGridConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }
};
}//ns
}//ns filters
#endif

