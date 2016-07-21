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
#include <stream_manipulator_3d/stream_manipulator.h>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/range/algorithm/copy.hpp>
#include <boost/bind.hpp>

namespace sm3d
{
StreamManipulator::StreamManipulator(const std::string ns):
    ROSNode<StreamManipulator>(ns), plugin_loader("stream_manipulator_3d", "sm3d::Plugin"),
    remover(),
    shm(128)
{}
StreamManipulator::~StreamManipulator(){}

void
StreamManipulator::deInit()
{
    //Process is stopped so clean the chain
    marks.reset();
    input.reset();
    output.reset();
    chain.clear(); //this cleans all the plugins
    sub->kill();
    sub.reset();
    cv_ptr.reset();
    new_cloud_in_stream.reset();
    mtx_stream_ptr.reset();
}

void
StreamManipulator::init()
{
    if(!nh){
        ROS_ERROR("[StreamManipulator::%s] NodeHandle not initialized, must call spawn() first!",__func__);
        return;
    }
    input = boost::make_shared<PTC>();
    output = boost::make_shared<PTC>();
    pub_markers = nh->advertise<visualization_msgs::MarkerArray>("markers", 1);
    marks = boost::make_shared<visualization_msgs::MarkerArray>();
    sub = boost::make_shared<Subscriber>(getPrivNodeHandle(), "subscriber");
    mtx_stream_ptr = boost::make_shared<boost::mutex>();
    cv_ptr = boost::make_shared<boost::condition_variable>();
    new_cloud_in_stream = boost::make_shared<bool>(false);
    sub->mtx_stream_ptr = mtx_stream_ptr;
    sub->cv_ptr = cv_ptr;
    sub->new_cloud_in_stream = new_cloud_in_stream;
    //Lock is necessary here because we  are writing into shared memory directly
    //by pushing back into chain_description, disabled and/or input topic
    ShmHandler::NamedLock lock(shm.mutex);
    //Save chain description in shared memory
    chain_description = shm.segment.construct<ShmHandler::StrVector> ("chain_description")(shm.str_alloc);
    if (nh->hasParam("chain_description")){
        std::vector<std::string> c_d;
        nh->getParam("chain_description",c_d);
        //write shared memory
        for(size_t i=0; i< c_d.size(); ++i){
            ShmHandler::String desc(shm.char_alloc);
            desc = c_d.at(i).c_str();
            chain_description->push_back(boost::move(desc));
            //Here desc is empty. Since desc  is allocated also in shared memory
            //it is  faster to move its  content to the new  string allocated in
            //the vector. No copy costructor is called
        }
    }
    //default is empty chain, so nothing has to be done about chain description
    //in shared memory, it was already allocated as an empty vector.

    //Read disabled flag
    bool dis(false);
    nh->param<bool>("disabled", dis, false);
    disabled_ = shm.segment.construct<bool> ("disabled")(dis);

    //Read input topic
    std::string inp_t;
    nh->param<std::string>("input_topic", inp_t, "/camera/depth_registered/points");
    input_topic = shm.segment.construct<ShmHandler::String>("input_topic")(shm.char_alloc);
    *input_topic = inp_t.c_str();

    //init chain_changed
    chain_changed = shm.segment.construct<bool>("chain_changed")(false);

    //init elapsed delay
    delay = shm.segment.construct<long>("delay")(0);
    ROS_INFO("[StreamManipulator::%s] Initialization complete",__func__);
}

//when service to get scene is called
/* bool */
/* BasicNode::cb_get_scene(pacman_vision_comm::get_scene::Request& req, pacman_vision_comm::get_scene::Response& res) */
/* { */
/*     //This saves in home... possible todo improvement to let user specify location */
/*     if (isDisabled()){ */
/*         ROS_WARN("[BasicNode::%s]\tNode is globally disabled, this service is suspended!",__func__); */
/*         return false; */
/*     } */
/*     if (scene_processed){ */
/*         sensor_msgs::PointCloud2 msg; */
/*         if (!req.save.empty()){ */
/*             pcl::PointCloud<pcl::PointXYZRGBA> cloud; */
/*             pcl::copyPointCloud(*scene_processed, cloud); */
/*             if(pcl::io::savePCDFileBinaryCompressed( req.save.c_str(), cloud) == 0) */
/*                 ROS_INFO("[BasicNode::%s]\tScene Processed saved to %s", __func__, req.save.c_str()); */
/*             else */
/*                 ROS_ERROR("[BasicNode::%s]\tFailed to save scene to %s", __func__, req.save.c_str()); */
/*         } */
/*         pcl::toROSMsg(*scene_processed, msg); */
/*         res.scene = msg; */
/*         return true; */
/*     } */
/*     else{ */
/*         ROS_WARN("[BasicNode::%s]\tNo Scene Processed to send to Service!", __func__); */
/*         return false; */
/*     } */
/* } */

/* void */
/* BasicNode::remove_outliers(const PTC::ConstPtr source, PTC::Ptr &dest) */
/* { */
/*     if (!dest) */
/*         dest=boost::make_shared<PTC>(); */
/*     pcl::StatisticalOutlierRemoval<PT> sor; */
/*     int k; */
/*     double std_mul; */
/*     config->get("outliers_mean_k", k); */
/*     config->get("outliers_std_mul", std_mul); */
/*     sor.setInputCloud(source); */
/*     sor.setMeanK(k); */
/*     sor.setStddevMulThresh(std_mul); */
/*     sor.filter(*dest); */
/* } */

/* void */
/* BasicNode::downsamp_scene(const PTC::ConstPtr source, PTC::Ptr &dest){ */
/*     //cannot keep organized cloud after voxelgrid */
/*     if (!dest) */
/*         dest=boost::make_shared<PTC>(); */
/*     pcl::VoxelGrid<PT> vg; */
/*     double leaf; */
/*     config->get("downsampling_leaf_size", leaf); */
/*     vg.setLeafSize( leaf, leaf, leaf); */
/*     vg.setDownsampleAllData(true); */
/*     vg.setInputCloud (source); */
/*     vg.filter (*dest); */
/* } */
/* void */
/* BasicNode::segment_scene(const PTC::ConstPtr source, PTC::Ptr &dest) */
/* { */
/*     if (!dest) */
/*         dest=boost::make_shared<PTC>(); */
/*     pcl::SACSegmentation<PT> seg; */
/*     pcl::ExtractIndices<PT> extract; */
/*     //coefficients */
/*     pcl::PointIndices::Ptr inliers (new pcl::PointIndices); */
/*     pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); */
/*     //plane segmentation */
/*     seg.setInputCloud(source); */
/*     seg.setOptimizeCoefficients (true); */
/*     seg.setModelType (pcl::SACMODEL_PLANE); */
/*     seg.setMethodType (pcl::SAC_RANSAC); */
/*     seg.setMaxIterations (100); */
/*     double tol; */
/*     config->get("plane_tolerance", tol); */
/*     seg.setDistanceThreshold (tol); */
/*     seg.segment(*inliers, *coefficients); */
/*     //extract what's on top of plane */
/*     extract.setInputCloud(seg.getInputCloud()); */
/*     extract.setIndices(inliers); */
/*     extract.setNegative(true); */
/*     extract.filter(*dest); */
/*     //optionally extract  a plane model  for visualization purpose and  create a */
/*     // marker from it */
/*     // Disabled, cause broken! */
/*      * if (config->publish_plane){ */
/*      *     extract.setNegative(false); */
/*      *     PTC::Ptr plane (new PTC); */
/*      *     extract.filter(*plane); */
/*      *     Eigen::Vector4f min,max; */
/*      *     pcl::getMinMax3D(*plane, min, max); */
/*      *     Box limits(min[0],min[1],min[2],max[0],max[1],max[2]); */
/*      *     create_box_marker(mark_plane, limits, true); */
/*      *     //make it red */
/*      *     mark_plane.color.g = 0.0f; */
/*      *     mark_plane.color.b = 0.0f; */
/*      *     //name it */
/*      *     mark_plane.ns="Plane Estimation Model"; */
/*      *     mark_plane.id=1; */
/*      *     mark_plane.header.frame_id = dest->header.frame_id; */
/*      * } */
/*      *1/ */
/* } */

// void
// BasicNode::extract_principal_color(const PTC::ConstPtr scene)
// {
//     //for now just compute the mean color...
//     //in the future we can create some palette and let the user choose
//     mean_L = mean_a = mean_b = 0.0;
//     for (const auto& pt: scene->points)
//     {
//         double L, a, b;
//         convertPCLColorToCIELAB(pt, L, a, b);
//         mean_L += L;
//         mean_a += a;
//         mean_b += b;
//     }
//     mean_L /= scene->size();
//     mean_a /= scene->size();
//     mean_b /= scene->size();
// }

/* void */
/* BasicNode::setFilterColor(const double r, const double g, const double b) */
/* { */
/*     convertRGBToCIELAB(r,g,b, ref_L, ref_a, ref_b); */
/* } */

/* void */
/* BasicNode::apply_color_filter(const PTC::ConstPtr source, PTC::Ptr &dest) */
/* { */
/*     if (!dest) */
/*         dest=boost::make_shared<PTC>(); */
/*     double thresh; */
/*     config->get("color_dist_thresh", thresh); */
/*     for (std::size_t i=0; i< source->size(); ++i) */
/*     { */
/*         double L,a,b; */
/*         convertPCLColorToCIELAB(source->points[i], L,a,b); */
/*         double dE = deltaE(ref_L, ref_a, ref_b, L,a,b); */
/*         bool invert(false); */
/*         config->get("invert_color_filter", invert); */
/*         if ( dE <= thresh && !invert) */
/*             dest->push_back(source->points[i]); */
/*         else if ( dE > thresh && invert) */
/*             dest->push_back(source->points[i]); */
/*     } */
/* } */
inline bool
StreamManipulator::checkChain() const
{
    return *chain_changed;
}

void
StreamManipulator::assembleChain()
{
    if (!nh)
        return;
    std::vector<sm3d::Plugin::Ptr> tmp_chain;
    //Lock the chain from processing thread
    Lock local(mtx_chain);
    //make a backup copy
    boost::copy(chain, std::back_inserter(tmp_chain));
    chain.clear();
    //have to lock also shared memory mutex for reading chain_description
    ShmHandler::NamedLock lock(shm.mutex);
    chain.resize(chain_description->size());
    //keep a record of wrong lines, to remove them from description
    std::vector<std::size_t> wrongs;
    for (std::size_t i=0; i<chain_description->size(); ++i)
    {
        std::string line = chain_description->at(i).c_str();
        boost::trim(line);
        std::vector<std::string> desc;
        boost::split(desc, line, boost::is_any_of(","), boost::token_compress_on);
        if (desc.size() != 2){
            ROS_ERROR("[StreamManipulator::%s] Chain description line is malformed: %s\nShould be 'customName,pluginName'\nSkipping this Plugin",__func__,line.c_str());
            wrongs.push_back(i);
            continue;
        }
        boost::trim(desc[0]);
        boost::trim(desc[1]);
        //Find if the Plugin was already present in old chain, in that case, copy it
        std::vector<std::string>::iterator pos  = std::find(
                old_chain_desc.begin(), old_chain_desc.end(), line);
        if (pos != old_chain_desc.end()){
            //Plugin was found, copy it from old chain
            chain[i] = tmp_chain.at(pos - old_chain_desc.begin());
        }
        else{
            //Plugin not found create it from scratch
            if (plugin_loader.isClassAvailable(desc[1])){
                chain[i] = plugin_loader.createInstance(desc[1]);
                chain[i]->init(desc[0], getNodeHandle(), getPrivNodeHandle());
            }
            else{
                ROS_ERROR("[StreamManipulator::%s] Attempted to load %s Plugin, but it does not exists...",__func__,desc[1].c_str());
                wrongs.push_back(i);
                continue;
            }
        }
    }
    //Remove wrong entries
    for (std::size_t i=0; i<wrongs.size(); ++i)
    {
        ShmHandler::StrVector::iterator wrong = chain_description->begin() + (wrongs[i] -i);
        chain_description->erase(wrong);
    }
    //reset chain changed
    *chain_changed = false;
    //save description for next iteration
    old_chain_desc.clear();
    for (std::size_t i=0; i<chain_description->size(); ++i)
        old_chain_desc.push_back(chain_description->at(i).c_str());
    marks.reset();
    ROS_INFO("[StreamManipulator::%s] Chain assembled",__func__);
}

void
StreamManipulator::process()
{
    if (!nh)
        return;
    if (!sub->isRunning())
        return;
    //Copy cloud on stream, but wait for it
    {
        boost::unique_lock<boost::mutex> lock(*mtx_stream_ptr);
        using namespace ::boost::posix_time;
        boost::system_time const timeout = boost::get_system_time() + boost::posix_time::milliseconds(20);
        //This version of timed_wait includes a loop, so we don't worry about spurious wakeups
        if (cv_ptr->timed_wait(lock, //lock to wait
                    timeout, //timeout
                    boost::bind(&StreamManipulator::streamPred, this)) ) //predicate evaluation
        {
            //someone notified during the wait, or predicate was set before the wait
            *new_cloud_in_stream = false;
            input = sub->stream;
            sub->stream.reset();
            frame_id = input->header.frame_id;
        }
        else
            //nothing new
            return;
    }
    if(input->empty()){
        ROS_WARN_THROTTLE(30,"[StreamManipulator::%s]\tEmpty input cloud, aborting...",__func__);
        return;
    }
    boost::posix_time::ptime pre = boost::posix_time::microsec_clock::local_time();
    //Lock locally the chain and process it. We dont want an assembleChain while
    //we are processing
    Lock local(mtx_chain);
    for (ChainIter it=chain.begin(); it!=chain.end(); ++it)
    {
        output = boost::make_shared<PTC>();
        (*it)->apply(input,output);
        input = output;
    }
    boost::posix_time::ptime post = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration elaps = post - pre;
    ShmHandler::NamedLock lock(shm.mutex);
    *delay = elaps.total_milliseconds();
}

void
StreamManipulator::publishMarkers() const
{
    if (marks && input && nh)
        if (pub_markers.getNumSubscribers()>0){
            for (size_t i=0; i<marks->markers.size(); ++i)
            {
                marks->markers[i].header.stamp = ros::Time::now();
                marks->markers[i].header.frame_id = frame_id;
                /* marks->markers[i].lifetime = ros::Duration(1.0); */
            }
            pub_markers.publish(*marks);
        }
}

void
StreamManipulator::assembleMarkers()
{
    if (!marks)
        marks = boost::make_shared<visualization_msgs::MarkerArray>();
    //We are reading the chain, thus we have to lock it
    Lock local(mtx_chain);
    for (ChainIter it=chain.begin(); it!=chain.end(); ++it)
    {
        bool has_mark(false);
        int found(-1);
        if ((*it)->hasMarker())
            has_mark=true;
        for (size_t i=0; i<marks->markers.size();++i)
        {
            if (marks->markers[i].ns.compare((*it)->getName())==0){
                found = i;
                break;
            }
        }
        if (has_mark){
            visualization_msgs::Marker m;
            (*it)->createMarker(m);
            if (found > -1)
                marks->markers.at(found) = m;
            else
                marks->markers.push_back(m);
        }
        else if (found > -1)
            marks->markers.erase(marks->markers.begin()+found);
    }
}

void
StreamManipulator::spinOnce()
{
    process();
}

//This is the processing thread
void
StreamManipulator::spin()
{
    while (isOk() && isRunning())
    {
        if (!isDisabled())
            spinOnce();
        queue_ptr->callAvailable(ros::WallDuration(0));
        sleep();
    }
}
void
StreamManipulator::spinMain(const double freq)
{
    ros::Rate main_rate(freq);
    assembleChain(); //do it one time only, then call it when checkChain is notified
    while (ros::ok() && isRunning())
    {
        {
            ShmHandler::NamedLock lock(shm.mutex);
            setDisabled(*disabled_);
            //pass input topic to subscriber
            sub->input_topic = input_topic->c_str();
        }
        if (!isDisabled()){
            if (!sub->isRunning())
                sub->spawn(50);
            if (checkChain())
                assembleChain();
            assembleMarkers();
            publishMarkers();
        }
        else if (sub->isRunning())
            sub->kill();
        ros::spinOnce();
        main_rate.sleep();
    }
}
}//namespace


