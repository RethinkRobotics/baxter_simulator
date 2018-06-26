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

#ifndef BAXTER_SIM_IO_BAXTER_IO_H
#define BAXTER_SIM_IO_BAXTER_IO_H

#include <QtWidgets/QMainWindow>
#include <QBitmap>
#include <QPushButton>
#include "ui_baxter_io.h"
#include "qnode.hpp"

namespace baxter_sim_io
{
class BaxterIO : public QMainWindow
{
  Q_OBJECT

public:
  BaxterIO(int argc, char** argv, QWidget* parent = 0);
  ~BaxterIO();

private:
  void setstyle(std::string, std::string, QPushButton&);

private Q_SLOTS:
  void on_left_arm_ok_pressed();
  void on_left_arm_ok_released();
  void on_left_arm_cancel_pressed();
  void on_left_arm_cancel_released();
  void on_left_arm_dial_sliderMoved(int position);
  void on_left_arm_show_pressed();
  void on_left_arm_show_released();
  void on_left_cuff_squeeze_pressed();
  void on_left_cuff_squeeze_released();
  void on_left_cuff_ok_pressed();
  void on_left_cuff_ok_released();
  void on_left_cuff_grasp_pressed();
  void on_left_cuff_grasp_released();
  void on_left_shoulder_ok_pressed();
  void on_left_shoulder_ok_released();
  void on_left_shoulder_cancel_pressed();
  void on_left_shoulder_cancel_released();
  void on_left_shoulder_show_pressed();
  void on_left_shoulder_show_released();
  void on_left_shoulder_dial_sliderMoved(int position);
  void on_right_shoulder_ok_pressed();
  void on_right_shoulder_ok_released();
  void on_right_shoulder_cancel_pressed();
  void on_right_shoulder_cancel_released();
  void on_right_shoulder_dial_sliderMoved(int position);
  void on_right_shoulder_show_pressed();
  void on_right_shoulder_show_released();
  void on_right_arm_ok_pressed();
  void on_right_arm_ok_released();
  void on_right_arm_cancel_pressed();
  void on_right_arm_cancel_released();
  void on_right_arm_dial_sliderMoved(int position);
  void on_right_arm_show_pressed();
  void on_right_arm_show_released();
  void on_right_cuff_squeeze_pressed();
  void on_right_cuff_squeeze_released();
  void on_right_cuff_ok_pressed();
  void on_right_cuff_ok_released();
  void on_right_cuff_grasp_pressed();
  void on_right_cuff_grasp_released();

private:
  Ui::BaxterIO* ui;
  QNode qnode;
};

}  // namespace baxter_sim_io

#endif  // BAXTER_SIM_IO_BAXTER_IO_H
