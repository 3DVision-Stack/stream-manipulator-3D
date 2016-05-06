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

#ifndef _SUBSCRIBER_HPP_
#define _SUBSCRIBER_HPP_

#include <stream_manipulator_3d/stream_manipulator.h>
#include <stream_manipulator_3d/common_ros.h>
#include <stream_manipulator_3d/common_pcl.h>
#include <stream_manipulator_3d/shared_memory_handler.hpp>
// ROS headers
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//PCL
#include <pcl/io/pcd_io.h>
#include <boost/asio.hpp>

namespace sm3d
{
class Subscriber: public ROSNode<Subscriber>
{
    friend class ROSNode<Subscriber>;
    friend class StreamManipulator;
    public:
        Subscriber(const ros::NodeHandle fn, const std::string ns): ROSNode<Subscriber>(fn,ns)
        {}
        typedef boost::shared_ptr<Subscriber> Ptr;
    protected:
        //init/deinit
        void init()
        {
            stream = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >();
            old_topic.clear();
            input_topic.clear();
            ROS_INFO("[%s::%s] Initialization complete",nh->getUnresolvedNamespace().c_str(),__func__);
        }
        void deInit()
        {
            old_topic.clear();
            stream.reset();
            ROS_INFO("[%s::%s] CleanUp complete",nh->getUnresolvedNamespace().c_str(),__func__);
        }
        void cb_stream(const sensor_msgs::PointCloud2::ConstPtr &msg)
        {
            boost::unique_lock<boost::mutex> lock(*mtx_stream_ptr);
            pcl::fromROSMsg (*msg, *stream);
            *new_cloud_in_stream = true;
            cv_ptr->notify_one();
        }
        void spin()
        {
            while (isOk() && isRunning())
            {
                if (input_topic.compare(old_topic) != 0){
                    //Topic changed, resubscribe
                    old_topic = input_topic;
                    stream_sub = nh->subscribe(nh->resolveName(old_topic),0,&Subscriber::cb_stream, this);
                    ROS_INFO("[%s::%s] Subscribed to %s",nh->getUnresolvedNamespace().c_str(),
                            __func__, nh->resolveName(old_topic).c_str());
                }
                spinOnce();
                sleep();
            }
        }
        //////////////////Members
        PTC::Ptr stream;
        ros::Subscriber stream_sub;
        std::string input_topic;
        std::string old_topic;
    private:
        //those are shared from StreamManipulator
        //protects the stream
        boost::shared_ptr<boost::mutex> mtx_stream_ptr;
        //conditional variable to notify a new cloud on stream
        boost::shared_ptr<boost::condition_variable>  cv_ptr;
        //wait condition
        boost::shared_ptr<bool> new_cloud_in_stream;
};
}//namespace
#endif
