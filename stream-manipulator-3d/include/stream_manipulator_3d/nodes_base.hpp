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
#ifndef _NODES_BASE_HPP_
#define _NODES_BASE_HPP_

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/package.h>
#include <ros/spinner.h>
#include <ros/callback_queue.h>
#include <ros/callback_queue_interface.h>

#include <stream_manipulator_3d/common_ros.h>
#include <stream_manipulator_3d/common_std.h>
//The magic starts here
///Base class  for dynamic nodes, should  be inherited in CRPT  fashion. See the
//end of the file for an example usage!
template<typename Derived>
class ROSNode
{
    public:
        //node with no name
        ROSNode()
        {
            derived().is_running = false;
            derived().disabled = false;
            derived().name.clear();
        }
        //ctor with  node handle to  create named NodeHandle
        //For top level node
        ROSNode(const std::string ns)
        {
            derived().is_running = false;
            derived().disabled = false;
            derived().name = ns;
        }
        //ctor with  node handle of  masternode, and child NodeHandle creation
        //This is for a child node, dependant of master node
        ROSNode(const ros::NodeHandle n, const std::string ns)
        {
            derived().is_running=false;
            derived().disabled=false;
            derived().name = ns;
            //copy construct father node handle
            derived().father_nh = boost::make_shared<ros::NodeHandle>(n);
        }
        virtual ~ROSNode()
        {
            derived().is_running=false;
            derived().worker.join();
            derived().nh->shutdown();
            derived().father_nh.reset();
            derived().name.clear();
        }
        bool inline isRunning() const
        {
            return derived().is_running;
        }
        bool inline isDisabled() const
        {
            return derived().disabled;
        }
        inline const std::string getNamespace() const
        {
            if(nh)
                return derived().nh->getNamespace();
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no namespace, it is not spawned yet...",__func__);
                return std::string();
            }
        }
        inline const std::string getFatherNamespace() const
        {
            if(derived().father_nh)
                return derived().father_nh->getNamespace();
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no father namespace, it's the top level one...",__func__);
                return std::string();
            }
        }
        ///wait for rate, or dont if it doesnt exist
        inline void sleep()
        {
            if (derived().spin_rate)
                derived().spin_rate->sleep();
        }
        //Check OK-ness
        inline bool isOk() const
        {
            if (derived().nh)
                return derived().nh->ok();
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no associated NodeHandle yet, it is not spawned, thus cannot check Ok-ness ...",__func__);
                return false;
            }
        }
        //get a copy of current nodehandle
        inline ros::NodeHandle getNodeHandle() const
        {
            if (derived().nh)
                return *(derived().nh);
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no associated NodeHandle yet, it is not spawned ...",__func__);
                return ros::NodeHandle();
            }
        }
        //get a copy  of father nodehandle, if  it exists or an empty  one if it
        //doesnt
        inline ros::NodeHandle getFatherNodeHandle() const
        {
            if (derived().father_nh)
                return *(derived().father_nh);
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no father NodeHandle, it is a top level one...",__func__);
                return ros::NodeHandle();
            }
        }
        //get a copy of the main NodeHandle if ti exists
        inline ros::NodeHandle getPrivNodeHandle() const
        {
            if (derived().priv_nh)
                return *(derived().priv_nh);
            else{
                ROS_WARN("[ROSNode::%s]\tNode has no private NodeHandle, set one with setPrivNodeHandle()",__func__);
                return ros::NodeHandle();
            }
        }
        //set the private NodeHandle
        void setPrivNodeHandle(const std::string name)
        {
            derived().priv_nh = boost::make_shared<ros::NodeHandle>(name);
        }
        //pause/unpause the node
        inline void setDisabled(bool disabled)
        {
            derived().disabled=disabled;
        }
        //get relative namespace of node
        inline std::string getName() const
        {
            return derived().name;
        }
        //kill the node for good
        void kill()
        {
            if (!derived().isRunning()){
                ROS_WARN("[ROSNode::%s]\tTried to kill %s, but it is not running.",__func__,derived().name.c_str());
                return;
            }
            derived().is_running = false;
            derived().worker.join();
            //Derived class should implement a deInit() function
            derived().deInit();
            derived().nh->shutdown();
            /* derived().nh.reset();  NO need to reset it, ros::shutdown takes care of it*/
            derived().queue_ptr.reset();
        }
        //spawn with spin implemented in Derived,  or if it doesnt exist use the
        //spin implemented here, also sets optional spin rate
        void spawn(const double frequency=0.0)
        {
            if(derived().isRunning()){
                ROS_WARN("[ROSNode::%s]\tTried to spawn %s, but it is already spawned and running.",__func__,derived().name.c_str());
                return;
            }
            if(frequency == 0.0)
                ROS_INFO("[ROSNode::%s]\tSpawning %s without a defined spin rate, this node will spin at maximum speed! Call spawn(double) if you don't want this!",__func__,derived().name.c_str());
            else
                derived().spin_rate = boost::make_shared<ros::Rate>(frequency);
            if(derived().father_nh){
                //child version, must have a name
                if(!derived().name.empty())
                    derived().nh = boost::make_shared<ros::NodeHandle>(*derived().father_nh, name);
                else{
                    ROS_ERROR("[ROSNode::%s]\tChild node doesnt have a name, it's ill constructed, cannot spawn it",__func__);
                    return;
                }
            }
            else if(!derived().father_nh){
                if(!derived().name.empty())
                    //spawn with a name
                    derived().nh = boost::make_shared<ros::NodeHandle>(name);
                else
                    //spawn without a name
                    derived().nh = boost::make_shared<ros::NodeHandle>();
            }
            else{
                ROS_ERROR("[ROSNode::%s]\t%s doesn't have neither a node handle nor a father nodehandle to depend onto, thus this Node is not validly constructed. Cannor spawn it...",__func__,derived().name.c_str());
                return;
            }
            derived().queue_ptr = boost::make_shared<ros::CallbackQueue>();
            derived().nh->setCallbackQueue(&(*queue_ptr));
            //Derived Class should implement init()
            derived().init();
            derived().is_running = true;
            derived().worker = boost::thread(&Derived::spin, derived_ptr());
        }
        //general spinOnce
        void spinOnce()
        {
            derived().queue_ptr->callAvailable(ros::WallDuration(0));
        }
    protected:
        //general spin, cannot be called from outside, only via spawn
        void spin()
        {
            //a  reimplementation of  spin should  check against  isRunning() to
            //correctly terminate the spinning thread
            while(derived().nh->ok() && derived().is_running)
            {
                if(!derived().disabled)
                    derived().spinOnce();
                derived().queue_ptr->callAvailable(ros::WallDuration(0));
                derived().sleep();
            }
        }
        void init()
        {
            ROS_WARN("[ROSNode::%s]\tThis Node did not reimplement an init() function. This one provided does nothing. Please implement one to suppress this warning!",__func__);
        }
        void deInit()
        {
            ROS_WARN("[ROSNode::%s]\tThis Node did not reimplement a deInit() function. This one provided does nothing. Please implement one to suppress this warning!",__func__);
        }
    public:
        //Additional spinMain loop, using global CallbackQueue, to block main
        void spinMain(const double freq=0)
        {
            ros::Rate main_rate(freq);
            while (ros::ok())
            {
                ros::spinOnce();
                main_rate.sleep();
            }
        }
    ///////Members
    protected:
        bool is_running;
        bool disabled;
        //threaded NodeHandle with custom CallbackQueue, spawn set this
        boost::shared_ptr<ros::NodeHandle> nh;
        //NodeHandle with default global CallbackQueue, optionally set with
        //setPrivNodeHandle
        boost::shared_ptr<ros::NodeHandle> priv_nh;
        //Copy of the father NodeHandle if it exists, for nodes created as child
        boost::shared_ptr<ros::NodeHandle> father_nh;
        boost::shared_ptr<ros::Rate> spin_rate;
        std::string name;
        boost::shared_ptr<ros::CallbackQueue> queue_ptr;
        boost::thread worker;
    private:
        //return derived ref
        Derived& derived() { return *static_cast<Derived*>(this); }
        const Derived& derived() const { return *static_cast<const Derived*>(this); }
        //return derived_ptr
        Derived* derived_ptr() {return static_cast<Derived*>(this); }
        const Derived* derived_ptr() const {return static_cast<const Derived*>(this); }
};
#endif

/** Example usage:
 *
class Foo: public ROSNode<Foo>
{
    public:
        friend class ROSNode<Foo>; //this lets the base class access all of our members, even private ones
        Foo (const std::string ns, const double rate):
            ROSNode<Foo>(ns,rate)
        {
        // Additional operations, like publishers/subscribers
        }
    protected:
        //redefine spin
        void spin()
        {
            while(isOk() && isRunning())
            {
                std::cout<<"Foo is spinning\n";
                spinOnce();
                sleep();
            }
        }
};
* main
int main (int argc, char *argv[])
{
    ros::init(argc, argv, ns);
    Foo node("foo", 40);
    node.spawn();
    int count (0);
    while (node.isOk() && node.isRunning())
    {
        //main node is spinning at 40Hz, its namespace is "foo"
        if (count == 200)
        {
            node.kill();
            //Foo node dies, it's NodeHandle  dies with it
            //This will also break the above loop
        }
        ++count;
    }
    return 0;
}
*/

