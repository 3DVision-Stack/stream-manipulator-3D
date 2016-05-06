/*
 * Software License Agreement (BSD License)
 *
 *   Copyright (c) 2016, Federico Spinelli (fspinelli@gmail.com)
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder(s) nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <rqt_stream_manipulator_3d/wait_for_dialog.h>
#include <QTimer>
#include <boost/interprocess/managed_shared_memory.hpp>

namespace rqt_sm3d
{
    WaitForDialog::WaitForDialog(QWidget *parent): QMessageBox(parent)
    {
        setIcon(QMessageBox::Critical);
        setText("Could not find Stream Manipulator Node running, please run it...");
        setWindowTitle("rqt_stream_manipulator: Waiting for node");
        setStandardButtons(QMessageBox::Cancel);
        QTimer *timer = new QTimer(this);
        connect( timer, SIGNAL( timeout() ), this, SLOT( onTimer() ));
        timer->start( 500 );
    }

    bool WaitForDialog::checkShm()
    {
        using namespace ::boost::interprocess;
        try
        {
            //check if chain_changed exists, if it does it means sm3d was initialized
            boost::interprocess::managed_shared_memory seg(
                    boost::interprocess::open_only, "sm3dMemory");
            if (!seg.find<bool>("chain_changed").first)
                return false;
        }
        catch (interprocess_exception &ex)
        {
            //memory not found
            return false;
        }
        //resources are there
        return true;
    }

    void WaitForDialog::onTimer()
    {
        if (checkShm())
            accept();
    }

}//End namespace
