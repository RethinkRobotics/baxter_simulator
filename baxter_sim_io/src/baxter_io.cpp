/*********************************************************************
 # Copyright (c) 2013-2015, Rethink Robotics
 # All rights reserved.
 #
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #
 # 1. Redistributions of source code must retain the above copyright notice,
 #    this list of conditions and the following disclaimer.
 # 2. Redistributions in binary form must reproduce the above copyright
 #    notice, this list of conditions and the following disclaimer in the
 #    documentation and/or other materials provided with the distribution.
 # 3. Neither the name of the Rethink Robotics nor the names of its
 #    contributors may be used to endorse or promote products derived from
 #    this software without specific prior written permission.
 #
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 # POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/**
 *  \author Hariharasudan Malaichamee
 *  \desc   Includes the action listener events for the QT controls
 */

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include <baxter_sim_io/baxter_io.hpp>

namespace baxter_sim_io
{
using namespace Qt;

void BaxterIO::setstyle(std::string pressed, std::string released, QPushButton& button)
{
  QPixmap pix(QString::fromUtf8(released.c_str()));
  button.setMask(pix.mask());
  button.setStyleSheet("QPushButton{background-image: url(" + QString::fromUtf8(released.c_str()) +
                       ");\n"
                       "background-position: center;\n"
                       "background-color: transparent;\n"
                       "background-repeat: none;\n"
                       "border: none;}\n"
                       "QPushButton:pressed{background-image: url(" +
                       QString::fromUtf8(pressed.c_str()) + ");}");
}

BaxterIO::BaxterIO(int argc, char** argv, QWidget* parent)
  : QMainWindow(parent), qnode(argc, argv), ui(new Ui::BaxterIO)
{
  ui->setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

  std::string cancel_r = ":/new/prefix/nav_cancel_r.png";
  std::string cancel_p = ":/new/prefix/nav_cancel_p.png";
  std::string ok_r = ":/new/prefix/nav_ok_r.png";
  std::string ok_p = ":/new/prefix/nav_ok_p.png";
  std::string show_r = ":/new/prefix/nav_show_r.png";
  std::string show_p = ":/new/prefix/nav_show_p.png";

  std::string cuff_grasp_r = ":/new/prefix/cuff_grasp_r.png";
  std::string cuff_grasp_p = ":/new/prefix/cuff_grasp_p.png";
  std::string cuff_ok_r = ":/new/prefix/cuff_ok_r.png";
  std::string cuff_ok_p = ":/new/prefix/cuff_ok_p.png";
  std::string cuff_squeeze_r = ":/new/prefix/cuff_squeeze_r.png";
  std::string cuff_squeeze_p = ":/new/prefix/cuff_squeeze_p.png";

  // Configure the stylesheet for all the Qwidgets
  // left arm
  setstyle(cancel_p, cancel_r, *(ui->left_arm_cancel));
  setstyle(ok_p, ok_r, *(ui->left_arm_ok));
  setstyle(show_p, show_r, *(ui->left_arm_show));
  // left shoulder
  setstyle(cancel_p, cancel_r, *(ui->left_shoulder_cancel));
  setstyle(ok_p, ok_r, *(ui->left_shoulder_ok));
  setstyle(show_p, show_r, *(ui->left_shoulder_show));
  // left cuff
  setstyle(cuff_grasp_p, cuff_grasp_r, *(ui->left_cuff_grasp));
  setstyle(cuff_ok_p, cuff_ok_r, *(ui->left_cuff_ok));
  setstyle(cuff_squeeze_p, cuff_squeeze_r, *(ui->left_cuff_squeeze));

  // right arm
  setstyle(cancel_p, cancel_r, *(ui->right_arm_cancel));
  setstyle(ok_p, ok_r, *(ui->right_arm_ok));
  setstyle(show_p, show_r, *(ui->right_arm_show));
  // right shoulder
  setstyle(cancel_p, cancel_r, *(ui->right_shoulder_cancel));
  setstyle(ok_p, ok_r, *(ui->right_shoulder_ok));
  setstyle(show_p, show_r, *(ui->right_shoulder_show));
  // right cuff
  setstyle(cuff_grasp_p, cuff_grasp_r, *(ui->right_cuff_grasp));
  setstyle(cuff_ok_p, cuff_ok_r, *(ui->right_cuff_ok));
  setstyle(cuff_squeeze_p, cuff_squeeze_r, *(ui->right_cuff_squeeze));
}

BaxterIO::~BaxterIO()
{
  delete ui;
}

void BaxterIO::on_left_arm_ok_pressed()
{
  QNode::left_arm_nav.buttons[0] = true;
}

void BaxterIO::on_left_arm_ok_released()
{
  QNode::left_arm_nav.buttons[0] = false;
}

void BaxterIO::on_left_arm_cancel_pressed()
{
  QNode::left_arm_nav.buttons[1] = true;
}

void BaxterIO::on_left_arm_cancel_released()
{
  QNode::left_arm_nav.buttons[1] = false;
}

void BaxterIO::on_left_arm_dial_sliderMoved(int position)
{
  QNode::left_arm_nav.wheel = position;
}

void BaxterIO::on_left_arm_show_pressed()
{
  QNode::left_arm_nav.buttons[2] = true;
}

void BaxterIO::on_left_arm_show_released()
{
  QNode::left_arm_nav.buttons[2] = false;
}

void BaxterIO::on_left_cuff_squeeze_pressed()
{
  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_left_cuff_squeeze_released()
{
  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void BaxterIO::on_left_cuff_ok_pressed()
{
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_left_cuff_ok_released()
{
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void BaxterIO::on_left_cuff_grasp_pressed()
{
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_left_cuff_grasp_released()
{
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void BaxterIO::on_left_shoulder_ok_pressed()
{
  QNode::left_shoulder_nav.buttons[0] = true;
}

void BaxterIO::on_left_shoulder_ok_released()
{
  QNode::left_shoulder_nav.buttons[0] = false;
}

void BaxterIO::on_left_shoulder_cancel_pressed()
{
  QNode::left_shoulder_nav.buttons[1] = true;
}

void BaxterIO::on_left_shoulder_cancel_released()
{
  QNode::left_shoulder_nav.buttons[1] = false;
}

void BaxterIO::on_left_shoulder_show_pressed()
{
  QNode::left_shoulder_nav.buttons[2] = true;
}

void BaxterIO::on_left_shoulder_show_released()
{
  QNode::left_shoulder_nav.buttons[2] = false;
}

void BaxterIO::on_left_shoulder_dial_sliderMoved(int position)
{
  QNode::left_shoulder_nav.wheel = position;
}

void BaxterIO::on_right_shoulder_ok_pressed()
{
  QNode::right_shoulder_nav.buttons[0] = true;
}

void BaxterIO::on_right_shoulder_ok_released()
{
  QNode::right_shoulder_nav.buttons[0] = false;
}

void BaxterIO::on_right_shoulder_cancel_pressed()
{
  QNode::right_shoulder_nav.buttons[1] = true;
}

void BaxterIO::on_right_shoulder_cancel_released()
{
  QNode::right_shoulder_nav.buttons[1] = false;
}

void BaxterIO::on_right_shoulder_dial_sliderMoved(int position)
{
  QNode::right_shoulder_nav.wheel = position;
}

void BaxterIO::on_right_shoulder_show_pressed()
{
  QNode::right_shoulder_nav.buttons[2] = true;
}

void BaxterIO::on_right_shoulder_show_released()
{
  QNode::right_shoulder_nav.buttons[2] = false;
}

void BaxterIO::on_right_arm_ok_pressed()
{
  QNode::right_arm_nav.buttons[0] = true;
}

void BaxterIO::on_right_arm_ok_released()
{
  QNode::right_arm_nav.buttons[0] = false;
}

void BaxterIO::on_right_arm_cancel_pressed()
{
  QNode::right_arm_nav.buttons[1] = true;
}

void BaxterIO::on_right_arm_cancel_released()
{
  QNode::right_arm_nav.buttons[1] = false;
}

void BaxterIO::on_right_arm_dial_sliderMoved(int position)
{
  QNode::right_arm_nav.wheel = position;
}

void BaxterIO::on_right_arm_show_pressed()
{
  QNode::right_arm_nav.buttons[2] = true;
}

void BaxterIO::on_right_arm_show_released()
{
  QNode::right_arm_nav.buttons[2] = false;
}

void BaxterIO::on_right_cuff_squeeze_pressed()
{
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_right_cuff_squeeze_released()
{
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void BaxterIO::on_right_cuff_ok_pressed()
{
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_right_cuff_ok_released()
{
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void BaxterIO::on_right_cuff_grasp_pressed()
{
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void BaxterIO::on_right_cuff_grasp_released()
{
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

}  // namespace baxter_sim_io
