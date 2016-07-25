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
#ifndef _FILTERS_FRUSTUM_HPP_
#define _FILTERS_FRUSTUM_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/filters/config/frustum_config.hpp>
#include <pcl/filters/frustum_culling.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

namespace sm3d
{
namespace filters
{
///Frustum Filter, wrapping pcl FrustumCulling.
class Frustum : public sm3d::Plugin
{
    public:
        virtual ~Frustum()
        {
            clean();
        }
        Frustum() : Plugin()
        {}
        //Yes  this plugin  has a  marker  and return  true  if it  needs to  be
        //published, it also must have a createMarker()
        virtual bool hasMarker()
        {
            ShmHandler::Lock  lock(config->mtx);
            return config->pub_marker;
        }

        ///initialize function
        virtual void init(const std::string &name, const ros::NodeHandle &father_nh,
                const ros::NodeHandle &priv_nh)
        {
            setNodeHandle(name, father_nh, priv_nh);
            //Then do our specific configuration
            //Create config in shared_memory
            config = shm.segment.construct<FrustumConfig>((name_+"Config").c_str())();

            //Lock the mutex to create parameters in shared memory (and in Rosparams)
            ShmHandler::Lock  lock(config->mtx);

            //////////////Init Parameters///////////////////////////////////////
            //Organized Flag, keeps point cloud organized when possible
            if (nh_->hasParam("organized"))
                nh_->getParam("organized", config->organized);
            else
                nh_->setParam("organized", config->organized);
            //Negative Flag, tells if the filter needs to remove points from the
            //box instead of keeping them
            if (nh_->hasParam("negative"))
                nh_->getParam("negative", config->negative);
            else
                nh_->setParam("negative", config->negative);
            //Pub_marker  Flag, if  true  create and  push a  box  marker to  be
            //published by SM3D
            if (nh_->hasParam("pub_marker"))
                nh_->getParam("pub_marker", config->pub_marker);
            else
                nh_->setParam("pub_marker", config->pub_marker);
            //Disabled flag, tells if the filter should be applied or not
            //Starts as disabled
            if (nh_->hasParam("disabled"))
                nh_->getParam("disabled", config->disabled);
            else
                nh_->setParam("disabled", config->disabled);
            nh_->deleteParam("near_plane_distance");
            nh_->deleteParam("far_plane_distance");
            //Frustum angles
            if (nh_->hasParam("horizontal_field_of_view"))
                nh_->getParam("horizontal_field_of_view",config->h_fov);
            else
                nh_->setParam("horizontal_field_of_view",config->h_fov);

            if (nh_->hasParam("vertical_field_of_view"))
                nh_->getParam("vertical_field_of_view",config->v_fov);
            else
                nh_->setParam("vertical_field_of_view",config->v_fov);

            //Distance of the frustum basis
            if (nh_->hasParam("near_plane_distance"))
                nh_->getParam("near_plane_distance",config->n_dist);
            else
                nh_->setParam("near_plane_distance",config->n_dist);

            if (nh_->hasParam("far_plane_distance"))
                nh_->getParam("far_plane_distance",config->f_dist);
            else
                nh_->setParam("far_plane_distance",config->f_dist);
            //Marker Color Vector R B G, actually makes sense only if pub marker
            //is true
            std::vector<double> c;
            if (nh_->hasParam("marker_color")){
                nh_->getParam("marker_color", c);
                if (c.size() != 3)
                    ROS_WARN("[%s::%s]\tInconsistent marker_color vector, resetting to default...",name_.c_str(),__func__);
                else{
                    config->color_r = c[0];
                    config->color_g = c[1];
                    config->color_b = c[2];
                }
            }
            else{
                c.resize(3);
                c[0]=config->color_r;
                c[1]=config->color_g;
                c[2]=config->color_b;
                nh_->setParam("marker_color",c);
            }
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
            ::pcl::FrustumCulling<PT> fc;
            fc.setKeepOrganized(config->organized);
            fc.setNegative(config->negative);
            fc.setHorizontalFOV(config->h_fov);
            fc.setVerticalFOV(config->v_fov);
            fc.setNearPlaneDistance(config->n_dist);
            fc.setFarPlaneDistance(config->f_dist);
            fc.setInputCloud(input);
            //Note: Camera pose is assumed to be a coordinate system where X is 
            //forward, Y is up, and Z is right.                                 
            Eigen::Matrix4f conversion;
            conversion <<   0,0,1,0,
                            0,-1,0,0,
                            1,0,0,0,
                            0,0,0,1;
            fc.setCameraPose(conversion);
            fc.filter (*output);
            output->header.frame_id = input->header.frame_id;
        }
        void createMarker(visualization_msgs::Marker &marker)
        {
            ShmHandler::Lock lock(config->mtx);
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.header.stamp = ros::Time::now();
            //adjust these values later if needed
            marker.ns = name_;
            marker.id = 0;
            marker.scale.x = 0.002;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = config->color_r;
            marker.color.g = config->color_g;
            marker.color.b = config->color_b;
            marker.color.a = 1.0f;
            marker.lifetime = ros::Duration(1);
            geometry_msgs::Point p, pf;
            double x_n = config->n_dist * std::tan( config->h_fov * M_PI/360);
            double y_n = config->n_dist * std::tan( config->v_fov * M_PI/360);
            double x_f = config->f_dist * std::tan( config->h_fov * M_PI/360);
            double y_f = config->f_dist * std::tan( config->v_fov * M_PI/360);
            //0-1
            p.x = x_n;
            p.y = y_n;
            p.z = config->n_dist;
            pf.x = -x_n;
            pf.y = y_n;
            pf.z = config->n_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //2-3
            pf.x = x_n;
            pf.y = -y_n;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //4-5
            pf.x = x_f;
            pf.y = y_f;
            pf.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //6-7
            p.x = -x_n;
            p.y = -y_n;
            p.z = config->n_dist;
            pf.x = -x_n;
            pf.y = y_n;
            pf.z = config->n_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //8-9
            pf.x = x_n;
            pf.y = -y_n;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //10-11
            pf.x = -x_f;
            pf.y = -y_f;
            pf.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //12-13
            p.x = -x_f;
            p.y = y_f;
            p.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //14-15
            pf.x = -x_n;
            pf.y = y_n;
            pf.z = config->n_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //16-17
            pf.x = x_f;
            pf.y = y_f;
            pf.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //18-19
            p.x = x_f;
            p.y = -y_f;
            p.z = config->f_dist;
            pf.x = x_f;
            pf.y = y_f;
            pf.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //20-21
            pf.x = x_n;
            pf.y = -y_n;
            pf.z = config->n_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //22-23
            pf.x = -x_f;
            pf.y = -y_f;
            pf.z = config->f_dist;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            config->mark_changed = false;
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
    ///////Members
    //  Configuration in shared memory
        FrustumConfig *config;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("organized");
            nh_->deleteParam("negative");
            nh_->deleteParam("pub_marker");
            nh_->deleteParam("disabled");
            nh_->deleteParam("horizontal_field_of_view");
            nh_->deleteParam("vertical_field_of_view");
            nh_->deleteParam("near_plane_distance");
            nh_->deleteParam("far_plane_distance");
            nh_->deleteParam("marker_color");
            shm.segment.destroy<FrustumConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }
};
}//ns
}//ns filters
#endif

