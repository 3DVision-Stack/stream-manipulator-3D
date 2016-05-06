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

#include <rqt_stream_manipulator_3d/rqt_stream_manipulator.h>
#include <rqt_stream_manipulator_3d/wait_for_dialog.h>
#include <pluginlib/class_list_macros.h>
#include <QFileDialog>
#include <QInputDialog>
#include <boost/filesystem.hpp>
#include <tinyxml.h>
#include <ros/master.h>

namespace rqt_sm3d
{

    StreamManipulator::StreamManipulator():
        rqt_gui_cpp::Plugin(), widget_(0), plugin_loader("rqt_stream_manipulator_3d", "rqt_sm3d::Plugin"),
        shm(128)
    {
        setObjectName("StreamManipulator");
    }

    void
    StreamManipulator::initPlugin(qt_gui_cpp::PluginContext& context)
    {
        widget_= new QWidget();
        ui_.setupUi(widget_);
        error_ = new QErrorMessage(widget_);
        if (context.serialNumber() > 1)
        {
            widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
        }
        context.addWidget(widget_);
        //Load Shared memory from stream_manipulator_3d, or wait for it to be launched
        WaitForDialog *dialog = new WaitForDialog(widget_);
        if (!dialog->checkShm())
            if (dialog->exec() != QDialog::Accepted )
            {
                //User pressed cancel, abort
                context.removeWidget(widget_);
                context.closePlugin();
                return;
            }
        //Load sm3d shared  memory parameters, no Lock is  necessary, cause find
        //and construct are async operations
        chain_description = shm.segment.find<ShmHandler::StrVector>("chain_description").first;
        disabled = shm.segment.find<bool>("disabled").first;
        input_topic = shm.segment.find<ShmHandler::String>("input_topic").first;
        chain_changed = shm.segment.find<bool>("chain_changed").first;
        if (!chain_description || !disabled || !input_topic || !chain_changed){
            //Error in loading shared memory, cannot continue
            error_->showMessage("Error in loading Stream Manipulator shared memory. Aborting...");
            context.removeWidget(widget_);
            context.closePlugin();
            return;
        }
        //init the gui
        onUpdateTopicList();
        getPlugins();
        fillListWidget();
        QStringList names;
        loadGuiPlugins(names);
        addTabs(names);
        connect(ui_.inputB_update, SIGNAL(pressed()), this, SLOT(onUpdateTopicList()));
        connect(ui_.input_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onChangeTopic(int)));
        connect(ui_.plugin_list, SIGNAL(currentTextChanged(const QString&)), this, SLOT(onPluginSelected(const QString&)));
        connect(ui_.controlB_add, SIGNAL(pressed()), this, SLOT(onAddPlugin()));
        connect(ui_.description_list, SIGNAL(itemSelectionChanged()), this, SLOT(onDescriptionSelected()));
        connect(ui_.controlB_del, SIGNAL(pressed()), this, SLOT(onDelPlugin()));
        connect(ui_.descriptionB_clear, SIGNAL(pressed()), this, SLOT(onClearPlugin()));
        connect(ui_.descriptionB_apply, SIGNAL(pressed()), this, SLOT(onUpdateChain()));
    }

    void
    StreamManipulator::shutdownPlugin()
    {
    }

    void
    StreamManipulator::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
    {
        //TODO
    }

    void
    StreamManipulator::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
    {
        //TODO
    }

    void
    StreamManipulator::fillListWidget()
    {
        //read chain_description in shared memory and fill the widget accordingly
        ui_.description_list->clear();
        ShmHandler::NamedLock lock(shm.mutex);
        for (std::size_t i=0; i< chain_description->size(); ++i)
        {
            QString entry = chain_description->at(i).c_str();
            ui_.description_list->addItem(entry);
        }
    }

    void
    StreamManipulator::onChangeTopic(int idx)
    {
        if (ui_.input_comboBox->currentIndex() != -1 && !ui_.input_comboBox->currentText().isEmpty()){
            ShmHandler::NamedLock lock(shm.mutex);
            *input_topic = ui_.input_comboBox->currentText().toStdString().c_str();
        }
    }

    void
    StreamManipulator::onUpdateTopicList()
    {
        //Disconnect the comboBox while we modify it
        disconnect(ui_.input_comboBox, 0, 0, 0);
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        QStringList qtopics;
        for (std::size_t i=0; i<topics.size(); ++i)
        {
            if (topics[i].datatype.compare("sensor_msgs/PointCloud2")==0)
                qtopics.append(topics[i].name.c_str());
        }
        ui_.input_comboBox->clear();
        ui_.input_comboBox->insertItems(666, qtopics);
        QString inp_topic;
        {
            ShmHandler::NamedLock  lock (shm.mutex);
            inp_topic = input_topic->c_str();
        }
        int i = qtopics.indexOf(inp_topic);
        if (i != -1)
            ui_.input_comboBox->setCurrentIndex(i);
        else{
            //Not found in comboBox, add the user specified topic to the list and select it
            ui_.input_comboBox->insertItem(ui_.input_comboBox->count(), inp_topic);
            ui_.input_comboBox->setCurrentIndex(ui_.input_comboBox->count() -1);
            //Color this entry Red so user knows this topic was not found
            ui_.input_comboBox->setItemData(ui_.input_comboBox->count()-1,
                    Qt::red, Qt::TextColorRole);
        }
        //Restore connection
        connect(ui_.input_comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onChangeTopic(int)));
    }

    void
    StreamManipulator::getPlugins()
    {
        std::string pack_name("stream_manipulator_3d");
        std::string plug("plugin");
        plugin_descriptions.clear();
        ros::package::V_string file_list;
        ros::package::getPlugins (pack_name, plug, file_list, true);
        QStringList plugins;
        for (size_t i=0; i<file_list.size(); ++i)
        {
            TiXmlDocument doc_plugin;
            if (doc_plugin.LoadFile(file_list[i])){
                TiXmlHandle handle(&doc_plugin);
                TiXmlElement* pLibTag = handle.FirstChild("library").ToElement();
                for (pLibTag; pLibTag; pLibTag=pLibTag->NextSiblingElement())
                {
                    TiXmlElement* pTag = pLibTag->FirstChild("class")->ToElement();
                    for (pTag; pTag; pTag=pTag->NextSiblingElement())
                    {
                        std::string desc = pTag->FirstChild("description")->ToElement()->GetText();
                        plugin_descriptions.push_back(desc);
                        plugins.append( pTag->Attribute("name") );
                    }
                }
            }
        }
        ui_.plugin_list->addItems(plugins);
    }

    void
    StreamManipulator::onAddPlugin()
    {
        bool ok;
        QString name = QInputDialog::getText(widget_, tr("QInputDialog::getText()"),
                                         tr("Provide a custom unique name for Plugin:"), QLineEdit::Normal,
                                         "MyCustomName", &ok);
        if (!ok || name.isEmpty())
            return;
        QString plugin = ui_.plugin_list->currentItem()->text();
        name.append(", ");
        name.append(plugin);
        ui_.description_list->addItem(name);
    }

    void
    StreamManipulator::onDescriptionSelected()
    {
        if (ui_.description_list->selectedItems().isEmpty())
            ui_.controlB_del->setDisabled(true);
        else
            ui_.controlB_del->setDisabled(false);
    }

    void
    StreamManipulator::onDelPlugin()
    {
        QList<QListWidgetItem*> selected = ui_.description_list->selectedItems();
        for (std::size_t i=0; i<selected.size(); ++i)
            delete ui_.description_list->takeItem(ui_.description_list->row(selected.at(i)));
    }

    void
    StreamManipulator::onClearPlugin()
    {
        ui_.description_list->clear();
    }

    void
    StreamManipulator::loadGuiPlugins(QStringList &names)
    {
        ui_.description_list->clearSelection();
        chain.clear();
        chain.resize(ui_.description_list->count());
        for (std::size_t i=0; i< ui_.description_list->count(); ++i)
        {
            QString entry = ui_.description_list->item(i)->text();
            QStringList list = entry.split(",", QString::SkipEmptyParts);
            QString name = list.first();
            QString plug = list[1];
            plug = plug.simplified();
            name = name.simplified();
            if (plugin_loader.isClassAvailable(plug.toStdString()))
                chain[i] = plugin_loader.createInstance(plug.toStdString());
            else{
                error_->showMessage(plug.append(": Plugin does not exist for loading, ignoring ..."));
                delete ui_.description_list->takeItem(i);
                --i;
                continue;
            }
            names.append(name);
            QIcon icon;
            QSize size(32,32);
            icon.addFile(":/icons/off.png", size); //Normal Off
            icon.addFile(":/icons/on.png", size, QIcon::Normal, QIcon::On); //Normal On
            name.prepend("  ");
            QPushButton *button = new QPushButton(icon,name);
            button->setCheckable(true);
            button->setIconSize(size);
            button->setStyleSheet("text-align: left");
            button->setFixedHeight(35);
            button->setFocusPolicy(Qt::NoFocus);
            chain[i]->button_ = button;
            ui_.chainVL->insertWidget(1+i,button);
        }
    }

    void
    StreamManipulator::addTabs(const QStringList &names)
    {
        for (std::size_t i=0; i<chain.size(); ++i)
        {
            chain[i]->init(names[i].toStdString()); //this will block until plugin is loaded
            ui_.tabWidget->insertTab(i+1, chain[i]->getWidgetPtr(), names[i]);
        }
    }

    void
    StreamManipulator::onUpdateChain()
    {
        //Clear old buttons layout
        //0 position is the label, we dont remove it ever
        for (std::size_t i =1; i < ui_.chainVL->count(); ++i)
        {
            QLayoutItem *child = ui_.chainVL->itemAt(i);
            //Remove until you find the spacer, which is last item, keeping it.
            if (child->spacerItem())
                break;
            else{
                ui_.chainVL->removeItem(child);
                delete child->widget();
                delete child;
                --i;
            }
        }
        //Remove old tabs
        //0 position is stream_manipulator, we dont touch it
        for (std::size_t i=1; i<ui_.tabWidget->count(); ++i)
        {
            delete ui_.tabWidget->widget(i);
            ui_.tabWidget->removeTab(i);
            --i;
        }
        QStringList names;
        loadGuiPlugins(names); //names are filled inside here
        {//Lock interprocess  mutex to update chain_description
            ShmHandler::NamedLock lock(shm.mutex);
            chain_description->clear();
            for (std::size_t i=0; i< ui_.description_list->count(); ++i)
            {
                QString entry = ui_.description_list->item(i)->text();
                ShmHandler::String desc(shm.char_alloc);
                desc = entry.toStdString().c_str();
                chain_description->push_back(boost::move(desc));
            }
            *chain_changed = true;
            shm.condition.notify_one(); //let ROS node build the chain of plugins
        }
        //Here we asyncronously continue to assemble the Guis
        addTabs(names);
    }

    void
    StreamManipulator::onPluginSelected(const QString &text)
    {
        if (text.isEmpty() || text.isNull()){
            ui_.bottomF->setDisabled(true);
            ui_.controlB_add->setDisabled(true);
            ui_.bottomL_name->clear();
            ui_.bottomL_description->clear();
        }
        else{
            ui_.bottomF->setDisabled(false);
            ui_.controlB_add->setDisabled(false);
            ui_.bottomL_name->setText(text);
            int index = ui_.plugin_list->currentIndex().row();
            ui_.bottomL_description->setText(plugin_descriptions[index].c_str());
        }
    }

    /* void */
    /* StreamManipulator::onRemovePlugin() */
    /* { */
    /*     //TODO */
    /* } */
}//End namespace

PLUGINLIB_EXPORT_CLASS(rqt_sm3d::StreamManipulator, rqt_gui_cpp::Plugin)
