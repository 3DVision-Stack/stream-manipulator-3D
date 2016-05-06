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

#ifndef _SHARED_MEMORY_HANDLER_HPP_
#define _SHARED_MEMORY_HANDLER_HPP_

#include <stream_manipulator_3d/common_std.h>
//shared memory management
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/named_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

//Instances, functions and typedefs to ease shared memory management
struct ShmHandler
{
    //define an interprocess vector of strings allocatable in shared memory
    typedef boost::interprocess::allocator<char,
            boost::interprocess::managed_shared_memory::segment_manager>
        CharAllocator; //allocator that can handle a Char in shm
    typedef boost::interprocess::basic_string<char, std::char_traits<char>, ShmHandler::CharAllocator>
        String; //basic string in shared  memory
    typedef boost::interprocess::allocator<ShmHandler::String,
            boost::interprocess::managed_shared_memory::segment_manager>
        StringAllocator; //Allocator for ShmString
    typedef boost::interprocess::vector<ShmHandler::String, ShmHandler::StringAllocator>
        StrVector; //Vector of strings

    //define the common shared memory lock on named mutex
    typedef boost::interprocess::scoped_lock<boost::interprocess::named_mutex> NamedLock;

    //define common shared memory lock on (unnamed) interprocess mutex
    typedef boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> Lock;

    //Construct shared memory (or open it). Also instantiate allocators
    ShmHandler(const std::size_t mega_bytes=64):
        segment(boost::interprocess::open_or_create, "sm3dMemory", mega_bytes*1024),
        mutex(boost::interprocess::open_or_create, "sm3dMutex"),
        condition(boost::interprocess::open_or_create, "sm3dCondition"),
        char_alloc(segment.get_segment_manager()), str_alloc(segment.get_segment_manager())
    {}
    virtual ~ShmHandler(){}

    //MEMBERS
    //shared memory segment for interprocess communication
    boost::interprocess::managed_shared_memory segment;
    //shared memory named mutex
    boost::interprocess::named_mutex mutex;
    //shared memory named condition
    boost::interprocess::named_condition condition;
    //allocators
    CharAllocator char_alloc;
    StringAllocator str_alloc;
};
#endif
