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

#ifndef _STREAM_MANIPULATOR_H_
#define _STREAM_MANIPULATOR_H_

#include <stream_manipulator_3d/nodes_base.hpp>
#include <stream_manipulator_3d/subscriber.hpp>
#include <stream_manipulator_3d/common_pcl.h>
#include <stream_manipulator_3d/common_ros.h>
#include <pluginlib/class_loader.h>
#include <stream_manipulator_3d/plugin.hpp>
#include <stream_manipulator_3d/shared_memory_handler.hpp>
// ROS headers
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <visualization_msgs/MarkerArray.h>
/* #include <tf/transform_broadcaster.h> */
/* #include <tf/transform_listener.h> */
// #include <pcl_ros/transforms.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl_conversions/pcl_conversions.h>
//PCL
#include <pcl/io/pcd_io.h>
// ROS generated headers
/* #include <stream_manipulator_3d/get_scene.h> */
//STD
#include <vector>
//Boost
#include <boost/asio.hpp>

namespace sm3d
{
class StreamManipulator: public ROSNode<StreamManipulator>
{
    friend class ROSNode<StreamManipulator>;
    public:
        StreamManipulator(const std::string ns);
        typedef boost::shared_ptr<StreamManipulator> Ptr;
        typedef  std::vector<sm3d::Plugin::Ptr>::iterator  ChainIter;
        //Redefined main spin
        void spinMain(const double freq=0);
        virtual ~StreamManipulator();
    protected:
        //init/deinit with ros params
        void init();
        void deInit();
        //Publish markers if requested
        void publishMarkers() const;
        //Assemble the markers to publish from various plugins
        void assembleMarkers();
        //Create a chain of filters that will process the input stream
        void assembleChain();
        //Check if the chain needs to be updated
        inline bool checkChain() const;
        //Main method to apply filters to input stream
        void process();
        //redefine spin and spinOnce
        void spin();
        void spinOnce();

        //////////////////Members
        //PluginLib class loader
        pluginlib::ClassLoader<sm3d::Plugin> plugin_loader;
        //publisher for markers
        ros::Publisher pub_markers;
        //marker to publish
        boost::shared_ptr<visualization_msgs::MarkerArray> marks;
        //input from subscriber
        PTC::Ptr input;
        //output from plugins
        PTC::Ptr output;
        //stream frame_id
        std::string frame_id;
        //Chain of Plugins as described by chain_description
        std::vector<sm3d::Plugin::Ptr> chain;
        //Actual Chain description before update
        std::vector<std::string> old_chain_desc;
        //subscriber
        Subscriber::Ptr sub;
    private:
        //convenient structure to remove shared memory on construction and destruction
        struct ShmRemover
        {
            ShmRemover()
            {
                using namespace ::boost::interprocess;
                shared_memory_object::remove("sm3dMemory");
                named_mutex::remove("sm3dMutex");
                named_condition::remove("sm3dCondition");
            }
            ~ShmRemover()
            {
                using namespace ::boost::interprocess;
                shared_memory_object::remove("sm3dMemory");
                named_mutex::remove("sm3dMutex");
                named_condition::remove("sm3dCondition");
            }
        } remover;
        //new cloud on strem predicat evaluation
        inline bool streamPred() const {return *new_cloud_in_stream;}
        //protects the stream within the process
        boost::shared_ptr<boost::mutex> mtx_stream_ptr;
        //conditional variable to notify a new cloud on stream, set by subscriber
        boost::shared_ptr<boost::condition_variable>  cv_ptr;
        //wait condition on stream, wrapping predicate
        boost::shared_ptr<bool> new_cloud_in_stream;
        //Mutex to protect the chain locally
        boost::mutex mtx_chain;
        //Shared memory handler
        ShmHandler shm;
        //////////////////////////////////////////PARAMETERS
        // (these are protected by sm3d Mutex in shared memory)
        //Chain of plugins description. Description is comma separated string of
        //user assigned name and plugin name, i.e.:
        //"MyCropBoxFilter, filters/cropbox" spaces dont  matter.
        //All user  assigned names must be  unique to ensure multiple  copies of
        //the same plugin can coexist.
        ShmHandler::StrVector *chain_description;
        //variable for chain changed predicate
        bool *chain_changed;
        //Disabled/Enabled flag
        bool *disabled_;
        //Input  topic, for subscriber to recieve input stream
        ShmHandler::String *input_topic;
};
}//namespace
#endif
