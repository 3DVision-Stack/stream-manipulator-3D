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

#ifndef _RQT_STREAM_MANIPULATOR_H_
#define _RQT_STREAM_MANIPULATOR_H_

#include <rqt_gui_cpp/plugin.h>
#include <ros/package.h>
#include <string>
#include <vector>
#include <deque>
#include <utility>
#include <pluginlib/class_loader.h>

#include <ui_rqt_stream_manipulator.h>

#include <QStringList>
#include <QString>
#include <QWidget>
#include <QErrorMessage>
#include <QTimer>

//Stream manipulator
#include <stream_manipulator_3d/shared_memory_handler.hpp>
#include <rqt_stream_manipulator_3d/plugin.h>


namespace rqt_sm3d
{

class StreamManipulator : public rqt_gui_cpp::Plugin
{
    Q_OBJECT

    //Methods
    public:
    StreamManipulator();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    /* virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const; */
    /* virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings); */

    protected:
    virtual void getPlugins();
    virtual void loadGuiPlugins(QStringList &names);
    virtual void addTabs(const QStringList &names);
    virtual void fillListWidget();

    protected slots:
    virtual void onUpdateTopicList();
    virtual void onChangeTopic(int idx);
    virtual void onPluginSelected(const QString &text);
    virtual void onDescriptionSelected();
    virtual void onDelPlugin();
    virtual void onAddPlugin();
    virtual void onClearPlugin();
    virtual void onUpdateChain();
    virtual void onAddSampleDelay();
    virtual void onUpdateDelay();
    virtual void onSaveConfig();
    virtual void onLoadConfig();
    virtual void onPauseResume(bool checked);

    //Members
    protected:
    Ui::StreamManipulatorWidget ui_;
    QWidget *widget_;
    QErrorMessage *error_; //simple pop-up window to display errors

    private:
    //time Measurements
    QTimer *timer;
    QTimer *delay_update_timer;
    std::deque<long> delays;
    //PluginLib class loader
    pluginlib::ClassLoader<rqt_sm3d::Plugin> plugin_loader;
    //Chain of GUI Plugins as described by chain_description
    std::vector<rqt_sm3d::Plugin::Ptr> chain;
    //description of each plugin
    ros::package::V_string plugin_descriptions;
    //Shared memory
    ShmHandler shm;
    ShmHandler::StrVector *chain_description;
    bool *chain_changed;
    bool *disabled;
    long *delay;
    ShmHandler::String *input_topic;
    bool *save, *load, *load_done;
    ShmHandler::String *save_path;
};
}//End namespace
#endif
