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
#ifndef _FILTERS_CROPBOX_HPP_
#define _FILTERS_CROPBOX_HPP_

#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/filters/config/cropbox_config.hpp>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

namespace sm3d
{
namespace filters
{
///CropBox Filter, wrapping pcl one.
class CropBox : public sm3d::Plugin
{
    public:
        virtual ~CropBox()
        {
            clean();
        }
        CropBox() : Plugin()
        {
            t_matrix.setIdentity();
        }
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
            config = shm.segment.construct<CropBoxConfig>((name_+"Config").c_str())();

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
            //Limits vector, define the cropbox limits |x,y,z|_min, |x,y,z|_max
            std::vector<double> lim;
            if (nh_->hasParam("limits")){
                nh_->getParam("limits",lim);
                if (lim.size() != 6)
                    ROS_WARN("[%s::%s]\tInconsistent filter limits, resetting to default...",name_.c_str(),__func__);
                else{
                    config->lim_x1 = lim[0];
                    config->lim_y1 = lim[1];
                    config->lim_z1 = lim[2];
                    config->lim_x2 = lim[3];
                    config->lim_y2 = lim[4];
                    config->lim_z2 = lim[5];
                }
            }
            else{
                lim.resize(6);
                lim[0]=config->lim_x1;
                lim[1]=config->lim_y1;
                lim[2]=config->lim_z1;
                lim[3]=config->lim_x2;
                lim[4]=config->lim_y2;
                lim[5]=config->lim_z2;
                nh_->setParam("limits",lim);
            }
            //Transform  vector, define  roto translation  of the point cloud
            // before the filter gets applied
            //   W X Y Z  tx ty tz
            std::vector<double> t;
            if (nh_->hasParam("transform")){
                nh_->getParam("transform", t);
                if (t.size() != 7)
                    ROS_WARN("[%s::%s]\tInconsistent transform vector, resetting to default...",name_.c_str(),__func__);
                else{
                    config->qw = t[0];
                    config->qx = t[1];
                    config->qy = t[2];
                    config->qz = t[3];
                    config->tx = t[4];
                    config->ty = t[5];
                    config->tz = t[6];
                }
            }
            else{
                t.resize(7);
                t[0]=config->qw;
                t[1]=config->qx;
                t[2]=config->qy;
                t[3]=config->qz;
                t[4]=config->tx;
                t[5]=config->ty;
                t[6]=config->tz;
                nh_->setParam("transform", t);
            }
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
            //Lock cropbox config mutex
            ShmHandler::Lock  lock(config->mtx);
            if (config->disabled){
                //Filter is disabled, just copy input into output
                output = input;
                return;
            }
            cb.setKeepOrganized(config->organized);
            cb.setNegative(config->negative);
            if (config->lim_changed){
                min[0] = config->lim_x1;
                min[1] = config->lim_y1;
                min[2] = config->lim_z1;
                max[0] = config->lim_x2;
                max[1] = config->lim_y2;
                max[2] = config->lim_z2;
                min[3]=max[3]=1.0;
                cb.setMin(min);
                cb.setMax(max);
                config->lim_changed=false;
            }
            if (config->trans_changed){
                Eigen::Quaternionf q(config->qw,config->qx,config->qy,config->qz);
                t_matrix.block(0,0,3,3) = q.toRotationMatrix();
                t_matrix.block(3,0,1,3) = Eigen::Vector3f::Zero().transpose();
                t_matrix(0,3) = config->tx;
                t_matrix(1,3) = config->ty;
                t_matrix(2,3) = config->tz;
                t_matrix(3,3) = 1.0;
                Eigen::Affine3f t (t_matrix);
                cb.setTransform(t);
                config->trans_changed=false;
            }
            cb.setInputCloud(input);
            cb.filter (*output);
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
            geometry_msgs::Pose pose;
            fromEigen(t_matrix.inverse(), pose);
            marker.pose = pose;
            //0-1
            p.x = config->lim_x1;
            p.y = config->lim_y1;
            p.z = config->lim_z1;
            pf.x = config->lim_x2;
            pf.y = config->lim_y1;
            pf.z = config->lim_z1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //2-3
            pf.x = config->lim_x1;
            pf.y = config->lim_y2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //4-5
            pf.y = config->lim_y1;
            pf.z = config->lim_z2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //6-7
            p.x = config->lim_x2;
            p.y = config->lim_y2;
            p.z = config->lim_z2;
            pf.x = config->lim_x2;
            pf.y = config->lim_y2;
            pf.z = config->lim_z1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //8-9
            pf.x = config->lim_x1;
            pf.z = config->lim_z2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //10-11
            pf.x = config->lim_x2;
            pf.y = config->lim_y1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //12-13
            p.x = config->lim_x1;
            p.y = config->lim_y1;
            p.z = config->lim_z2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //14-15
            pf.x = config->lim_x1;
            pf.y = config->lim_y2;
            pf.z = config->lim_z2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //16-17
            p.x = config->lim_x2;
            p.y = config->lim_y2;
            p.z = config->lim_z1;
            pf.x = config->lim_x2;
            pf.y = config->lim_y1;
            pf.z = config->lim_z1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //18-19
            pf.x = config->lim_x1;
            pf.y = config->lim_y2;
            pf.z = config->lim_z1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //20-21
            p.x = config->lim_x1;
            p.y = config->lim_y2;
            p.z = config->lim_z2;
            marker.points.push_back(p);
            marker.points.push_back(pf);
            //22-23
            p.x = config->lim_x2;
            p.y = config->lim_y1;
            p.z = config->lim_z2;
            pf.x = config->lim_x2;
            pf.y = config->lim_y1;
            pf.z = config->lim_z1;
            marker.points.push_back(p);
            marker.points.push_back(pf);
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
    ///////Members
    //  Configuration in shared memory
        CropBoxConfig *config;
        //Pcl cropbox obj
        ::pcl::CropBox<PT> cb;
        //Stored CropBox transform
        ::Eigen::Matrix4f t_matrix;
        //and limits
        ::Eigen::Vector4f min,max;
        //clean Rosparams and shared_memory
        void clean()
        {
            nh_->deleteParam("organized");
            nh_->deleteParam("negative");
            nh_->deleteParam("pub_marker");
            nh_->deleteParam("disabled");
            nh_->deleteParam("limits");
            nh_->deleteParam("transform");
            nh_->deleteParam("marker_color");
            shm.segment.destroy<CropBoxConfig>((name_+"Config").c_str());
            ROS_INFO("[%s::%s] CleanUp complete",name_.c_str(),__func__);
        }
};
}//ns
}//ns filters
#endif

