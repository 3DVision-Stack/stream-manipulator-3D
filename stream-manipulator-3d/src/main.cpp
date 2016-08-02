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
#include <stream_manipulator_3d/stream_manipulator.h>
#include <signal.h>

static sm3d::StreamManipulator *psm3d;

void sm3dSigintHandler(int sig)
{
    //This will  bring down  everything, removing the  shared memory  via deInit
    //call, subscriber is also killed inside the same function
    ROS_INFO("[StreamManipulator] Shutting Down...");
    psm3d->kill();
    ROS_INFO("[StreamManipulator] Goodbye!");
    ros::shutdown();
}

/*************** MAIN *********************/
int main (int argc, char *argv[])
{
    std::string node_name("stream_manipulator");
    ros::init(argc, argv, node_name);//, ros::init_options::NoSigintHandler);
    sm3d::StreamManipulator node(node_name);
    node.setPrivNodeHandle("~");
    node.spawn(50); //Now a node handle exists
    //We can install our own SIGINT handler
    psm3d = &node;
    signal(SIGINT, sm3dSigintHandler);
    //Blocking Call
    node.spinMain(10);
    return 1;
}
