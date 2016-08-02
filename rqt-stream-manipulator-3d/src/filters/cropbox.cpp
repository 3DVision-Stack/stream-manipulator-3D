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
#include <rqt_stream_manipulator_3d/filters/cropbox.h>
#include <pluginlib/class_list_macros.h>
#include <QColorDialog>

namespace rqt_sm3d
{
namespace filters
{
    void
    CropBox::init(const std::string &name)
    {
        name_ = name;
        page_= new QWidget();
        ui_.setupUi(page_);
        config = shm.segment.find<Config>((name_+"Config").c_str()).first;
        while(!config)
        {
            boost::this_thread::sleep(boost::posix_time::milliseconds(50));
            config = shm.segment.find<Config>((name_+"Config").c_str()).first;
            /* QCoreApplication::processEvents(QEventLoop::AllEvents, 100); */
        }
        {//Lock configuration mutex to read
            ShmHandler::Lock  lock(config->mtx);
            button_->setChecked(!config->disabled);
            if (!config->disabled)
                ui_.Title->setStyleSheet("background-color: green");
            ui_.pub_markerB->setChecked(config->pub_marker);
            ui_.organizedB->setChecked(config->organized);
            ui_.negativeB->setChecked(config->negative);
            ui_.Xmin->setValue(config->lim_x1);
            ui_.Xmax->setValue(config->lim_x2);
            ui_.Ymin->setValue(config->lim_y1);
            ui_.Ymax->setValue(config->lim_y2);
            ui_.Zmin->setValue(config->lim_z1);
            ui_.Zmax->setValue(config->lim_z2);
            ui_.QW->setValue(config->qw);
            ui_.QX->setValue(config->qx);
            ui_.QY->setValue(config->qy);
            ui_.QZ->setValue(config->qz);
            ui_.TX->setValue(config->tx);
            ui_.TY->setValue(config->ty);
            ui_.TZ->setValue(config->tz);
            QColor mcolor;
            mcolor.setRgbF(config->color_r, config->color_g, config->color_b);
            QPixmap pixm(32,32);
            pixm.fill(mcolor);
            QIcon icon(pixm);
            ui_.ColorSelectB->setIcon(icon);
            ui_.ColorSelectB->setIconSize(QSize(32,32));
        }
        ui_.Title->setText(ui_.Title->text().prepend(name_.c_str()));
        connect(button_, SIGNAL(clicked(bool)), this, SLOT(onEnableDisable(bool)));
        connect(ui_.Xmin, SIGNAL(valueChanged(double)), this, SLOT(onXminChanged(double)));
        connect(ui_.Ymin, SIGNAL(valueChanged(double)), this, SLOT(onYminChanged(double)));
        connect(ui_.Zmin, SIGNAL(valueChanged(double)), this, SLOT(onZminChanged(double)));
        connect(ui_.Xmax, SIGNAL(valueChanged(double)), this, SLOT(onXmaxChanged(double)));
        connect(ui_.Ymax, SIGNAL(valueChanged(double)), this, SLOT(onYmaxChanged(double)));
        connect(ui_.Zmax, SIGNAL(valueChanged(double)), this, SLOT(onZmaxChanged(double)));
        connect(ui_.QW, SIGNAL(valueChanged(double)), this, SLOT(onQWChanged(double)));
        connect(ui_.QX, SIGNAL(valueChanged(double)), this, SLOT(onQXChanged(double)));
        connect(ui_.QY, SIGNAL(valueChanged(double)), this, SLOT(onQYChanged(double)));
        connect(ui_.QZ, SIGNAL(valueChanged(double)), this, SLOT(onQZChanged(double)));
        connect(ui_.TX, SIGNAL(valueChanged(double)), this, SLOT(onTXChanged(double)));
        connect(ui_.TY, SIGNAL(valueChanged(double)), this, SLOT(onTYChanged(double)));
        connect(ui_.TZ, SIGNAL(valueChanged(double)), this, SLOT(onTZChanged(double)));
        connect(ui_.pub_markerB, SIGNAL(clicked(bool)), this, SLOT(onPubMarks(bool)));
        connect(ui_.organizedB, SIGNAL(clicked(bool)), this, SLOT(onOrganized(bool)));
        connect(ui_.negativeB, SIGNAL(clicked(bool)), this, SLOT(onNegative(bool)));
        connect(ui_.ColorSelectB, SIGNAL(clicked()), this, SLOT(onColorSelect()));
    }

    void
    CropBox::onEnableDisable(bool checked)
    {
        if (checked)
            ui_.Title->setStyleSheet("background-color: green");
        else
            ui_.Title->setStyleSheet("background-color: red");
        ShmHandler::Lock lock(config->mtx);
        config->disabled=!checked;
    }

    void
    CropBox::onNegative(bool checked)
    {
        ShmHandler::Lock lock(config->mtx);
        config->negative=checked;
    }

    void
    CropBox::onOrganized(bool checked)
    {
        ShmHandler::Lock lock(config->mtx);
        config->organized=checked;
    }

    void
    CropBox::onPubMarks(bool checked)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->pub_marker=checked;
    }

    void
    CropBox::onXminChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_x1 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onYminChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_y1 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onZminChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_z1 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onXmaxChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_x2 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onYmaxChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_y2 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onZmaxChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->lim_z2 = val;
        config->lim_changed = true;
    }
    void
    CropBox::onQWChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->qw = val;
        config->trans_changed = true;
    }
    void
    CropBox::onQXChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->qx = val;
        config->trans_changed = true;
    }
    void
    CropBox::onQYChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->qy = val;
        config->trans_changed = true;
    }
    void
    CropBox::onQZChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->qz = val;
        config->trans_changed = true;
    }
    void
    CropBox::onTXChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->tx = val;
        config->trans_changed = true;
    }
    void
    CropBox::onTYChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->ty = val;
        config->trans_changed = true;
    }
    void
    CropBox::onTZChanged(double val)
    {
        ShmHandler::Lock  lock(config->mtx);
        config->tz = val;
        config->trans_changed = true;
    }

    void
    CropBox::onColorSelect()
    {
        QColor color = QColorDialog::getColor(Qt::white, page_);
        if (!color.isValid())
            return;
        QPixmap pixm(32,32);
        pixm.fill(color);
        QIcon icon(pixm);
        ui_.ColorSelectB->setIcon(icon);
        ui_.ColorSelectB->setIconSize(QSize(32,32));
        ShmHandler::Lock  lock(config->mtx);
        config->color_r = color.redF();
        config->color_g = color.greenF();
        config->color_b = color.blueF();
    }
}//ns
}//ns filters

PLUGINLIB_EXPORT_CLASS(rqt_sm3d::filters::CropBox, rqt_sm3d::Plugin);
