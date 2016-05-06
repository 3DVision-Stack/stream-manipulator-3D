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
#include <stream_manipulator_3d/common_ros.h>

namespace sm3d
{
void
fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    dest.orientation.x = quad.x();
    dest.orientation.y = quad.y();
    dest.orientation.z = quad.z();
    dest.orientation.w = quad.w();
    dest.position.x = source(0,3);
    dest.position.y = source(1,3);
    dest.position.z = source(2,3);
}
void
fromEigen(const Eigen::Matrix4f &source, tf::Transform &dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
    dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
    dest.setRotation(q);
}
void
fromEigen(const Eigen::Matrix4f &source, geometry_msgs::Pose &dest, tf::Transform &tf_dest)
{
    Eigen::Matrix3f rot;
    rot<<source(0,0), source(0,1), source(0,2),
        source(1,0), source(1,1), source(1,2),
        source(2,0), source(2,1), source(2,2);
    Eigen::Quaternionf quad(rot);
    quad.normalize();
    dest.orientation.x = quad.x();
    dest.orientation.y = quad.y();
    dest.orientation.z = quad.z();
    dest.orientation.w = quad.w();
    dest.position.x = source(0,3);
    dest.position.y = source(1,3);
    dest.position.z = source(2,3);
    tf::Quaternion q(quad.x(), quad.y(), quad.z(), quad.w());
    tf_dest.setOrigin(tf::Vector3(source(0,3), source(1,3), source(2,3)));
    tf_dest.setRotation(q);
}

void
fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest, tf::Transform &tf_dest)
{
    Eigen::Quaternionf q(source.orientation.w, source.orientation.x, source.orientation.y, source.orientation.z);
    q.normalize();
    Eigen::Vector3f t(source.position.x, source.position.y, source.position.z);
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
    tf::Quaternion qt(q.x(), q.y(), q.z(), q.w());
    tf_dest.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    tf_dest.setRotation(qt);
}
void
fromPose(const geometry_msgs::Pose &source, Eigen::Matrix4f &dest)
{
    Eigen::Quaternionf q(source.orientation.w, source.orientation.x, source.orientation.y, source.orientation.z);
    q.normalize();
    Eigen::Vector3f t(source.position.x, source.position.y, source.position.z);
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
}
void
fromPose(const geometry_msgs::Pose &source, tf::Transform &dest)
{
    tf::Quaternion q(source.orientation.x, source.orientation.y, source.orientation.z, source.orientation.w);
    dest.setOrigin(tf::Vector3(source.position.x, source.position.y, source.position.z));
    dest.setRotation(q);
}

void
fromTF(const tf::Transform &source, Eigen::Matrix4f &dest, geometry_msgs::Pose &pose_dest)
{
    Eigen::Quaternionf q(source.getRotation().getW(),source.getRotation().getX(), source.getRotation().getY(),source.getRotation().getZ());
    q.normalize();
    Eigen::Vector3f t(source.getOrigin().x(), source.getOrigin().y(), source.getOrigin().z());
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
    pose_dest.orientation.x = q.x();
    pose_dest.orientation.y = q.y();
    pose_dest.orientation.z = q.z();
    pose_dest.orientation.w = q.w();
    pose_dest.position.x = t(0);
    pose_dest.position.y = t(1);
    pose_dest.position.z = t(2);
}
void
fromTF(const tf::Transform &source, Eigen::Matrix4f &dest)
{
    Eigen::Quaternionf q(source.getRotation().getW(),source.getRotation().getX(), source.getRotation().getY(),source.getRotation().getZ());
    q.normalize();
    Eigen::Vector3f t(source.getOrigin().x(), source.getOrigin().y(), source.getOrigin().z());
    Eigen::Matrix3f R(q.toRotationMatrix());
    dest(0,0) = R(0,0);
    dest(0,1) = R(0,1);
    dest(0,2) = R(0,2);
    dest(1,0) = R(1,0);
    dest(1,1) = R(1,1);
    dest(1,2) = R(1,2);
    dest(2,0) = R(2,0);
    dest(2,1) = R(2,1);
    dest(2,2) = R(2,2);
    dest(3,0) = dest(3,1)= dest(3,2) = 0;
    dest(3,3) = 1;
    dest(0,3) = t(0);
    dest(1,3) = t(1);
    dest(2,3) = t(2);
}
void
fromTF(const tf::Transform &source, geometry_msgs::Pose &dest)
{
    dest.orientation.x = source.getRotation().getX();
    dest.orientation.y = source.getRotation().getY();
    dest.orientation.z = source.getRotation().getZ();
    dest.orientation.w = source.getRotation().getW();
    dest.position.x = source.getOrigin().x();
    dest.position.y = source.getOrigin().y();
    dest.position.z = source.getOrigin().z();
}
/* void */
/* create_box_marker(const Box lim, visualization_msgs::Marker &marker, const bool cube_type) */
/* { */
/*     if(!cube_type){ */
/*         // Create lines */
/*         //Does not set time header, ref_frame, namespace and id of marker */
/*         marker.type = visualization_msgs::Marker::LINE_LIST; */
/*         marker.header.stamp = ros::Time(); */
/*         //adjust these two values later if needed */
/*         marker.ns = "box"; */
/*         marker.id = 0; */
/*         marker.scale.x = 0.002; */
/*         marker.action = visualization_msgs::Marker::ADD; */
/*         //white color default */
/*         marker.color.r = 1.0f; */
/*         marker.color.g = 1.0f; */
/*         marker.color.b = 1.0f; */
/*         marker.color.a = 1.0f; */
/*         //pose Identity */
/*         marker.pose.position.x=0.0f; */
/*         marker.pose.position.y=0.0f; */
/*         marker.pose.position.z=0.0f; */
/*         marker.pose.orientation.x=0.0f; */
/*         marker.pose.orientation.y=0.0f; */
/*         marker.pose.orientation.z=0.0f; */
/*         marker.pose.orientation.w=1.0f; */
/*         marker.lifetime = ros::Duration(1); */
/*         geometry_msgs::Point p, pf; */
/*         //0-1 */
/*         p.x = lim.x1; */
/*         p.y = lim.y1; */
/*         p.z = lim.z1; */
/*         pf.x = lim.x2; */
/*         pf.y = lim.y1; */
/*         pf.z = lim.z1; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //2-3 */
/*         pf.x = lim.x1; */
/*         pf.y = lim.y2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //4-5 */
/*         pf.x = lim.x1; */
/*         pf.y = lim.y1; */
/*         pf.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //6-7 */
/*         p.x = lim.x2; */
/*         p.y = lim.y2; */
/*         p.z = lim.z2; */
/*         pf.x = lim.x2; */
/*         pf.y = lim.y2; */
/*         pf.z = lim.z1; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //8-9 */
/*         pf.x = lim.x1; */
/*         pf.y = lim.y2; */
/*         pf.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //10-11 */
/*         pf.x = lim.x2; */
/*         pf.y = lim.y1; */
/*         pf.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //12-13 */
/*         p.x = lim.x1; */
/*         p.y = lim.y1; */
/*         p.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //14-15 */
/*         pf.x = lim.x1; */
/*         pf.y = lim.y2; */
/*         pf.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //16-17 */
/*         p.x = lim.x2; */
/*         p.y = lim.y2; */
/*         p.z = lim.z1; */
/*         pf.x = lim.x2; */
/*         pf.y = lim.y1; */
/*         pf.z = lim.z1; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //18-19 */
/*         pf.x = lim.x1; */
/*         pf.y = lim.y2; */
/*         pf.z = lim.z1; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //20-21 */
/*         p.x = lim.x1; */
/*         p.y = lim.y2; */
/*         p.z = lim.z2; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*         //22-23 */
/*         p.x = lim.x2; */
/*         p.y = lim.y1; */
/*         p.z = lim.z2; */
/*         pf.x = lim.x2; */
/*         pf.y = lim.y1; */
/*         pf.z = lim.z1; */
/*         marker.points.push_back(p); */
/*         marker.points.push_back(pf); */
/*     } */
/*     else{ */
/*         // create a cube */
/*         //Does not set time header, ref_frame, namespace and id of marker */
/*         marker.type = visualization_msgs::Marker::CUBE; */
/*         marker.header.stamp = ros::Time(); */
/*         //adjust these two values later if needed */
/*         marker.ns = "box"; */
/*         marker.id = 0; */
/*         marker.scale.x = (lim.x2-lim.x1)*0.5; */
/*         marker.scale.y = (lim.y2-lim.y1)*0.5; */
/*         marker.scale.z = (lim.z2-lim.z1)*0.5; */
/*         marker.action = visualization_msgs::Marker::ADD; */
/*         //white color default, transparent */
/*         marker.color.r = 1.0f; */
/*         marker.color.g = 1.0f; */
/*         marker.color.b = 1.0f; */
/*         marker.color.a = 0.3f; */
/*         //pose Identity */
/*         marker.pose.position.x=0.0f; */
/*         marker.pose.position.y=0.0f; */
/*         marker.pose.position.z=0.0f; */
/*         marker.pose.orientation.x=0.0f; */
/*         marker.pose.orientation.y=0.0f; */
/*         marker.pose.orientation.z=0.0f; */
/*         marker.pose.orientation.w=1.0f; */
/*         marker.points.clear(); */
/*         geometry_msgs::Point p; */
/*         p.x = (lim.x2+lim.x1)*0.5; */
/*         p.y = (lim.y2+lim.y1)*0.5; */
/*         p.z = (lim.z2+lim.z1)*0.5; */
/*         marker.points.push_back(p); */
/*         marker.lifetime = ros::Duration(1); */
/*     } */
/* } */
} //namespace
