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
#ifndef _COMMON_ROS_H_
#define _COMMON_ROS_H_
// ROS headers
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

namespace sm3d
{
/// Convert from eigen 4x4 matrix to tf and pose
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest);
/// Convert from eigen 4x4 matrix to pose
void fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest);
/// convert from eigen 4x4 matrix to tf
void fromEigen(const Eigen::Matrix4f &source, tf::Transform &dest);
/// Convert from pose to eigen 4x4 matrix and tf
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest);
/// Convert from pose to eigen 4x4 matrix
void fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest);
/// Convert from pose to tf
void fromPose(const geometry_msgs::Pose &source, tf::Transform &dest);
/// Convert  from  tf  to  eigen  4x4   matrix  and  pose
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest);
/// Convert  from  tf  to  eigen  4x4   matrix
void fromTF(const tf::Transform &source, Eigen::Matrix4f &dest);
/// Convert  from  tf  to pose
void fromTF(const tf::Transform &source, geometry_msgs::Pose &dest);

/*! \brief Create  a box  marker out  of a  Box object.
 *
 * \param[in] lim Box object containing the limits the marker will have.
 * \param[out] marker Corresponding marker created from lim.
 * \param[in] cube_type if true create a semitransparent cube instead of a collection of lines.
 * \note Does not set frame_id.
 */
//void create_box_marker(const Box lim, visualization_msgs::Marker &marker, const bool cube_type=false);

}//namespace
#endif
