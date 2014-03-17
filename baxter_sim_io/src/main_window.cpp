/*********************************************************************
 # Copyright (c) 2014, Rethink Robotics
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
#include <baxter_sim_io/main_window.hpp>

namespace baxter_sim_io {

using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent),
      qnode(argc, argv),
      ui(new Ui::MainWindow) {
  ui->setupUi(this);  // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

}

MainWindow::~MainWindow() {
}

void MainWindow::on_left_arm_ok_pressed() {
  QNode::left_arm_nav.buttons[0] = true;
}

void MainWindow::on_left_arm_ok_released() {
  QNode::left_arm_nav.buttons[0] = false;
}

void MainWindow::on_left_arm_cancel_pressed() {
  QNode::left_arm_nav.buttons[1] = true;
}

void MainWindow::on_left_arm_cancel_released() {
  QNode::left_arm_nav.buttons[1] = false;
}

void MainWindow::on_left_arm_dial_sliderMoved(int position) {
  QNode::left_arm_nav.wheel = position;
}

void MainWindow::on_left_arm_show_pressed() {
  QNode::left_arm_nav.buttons[2] = true;
}

void MainWindow::on_left_arm_show_released() {
  QNode::left_arm_nav.buttons[2] = false;
}

void MainWindow::on_left_cuff_squeeze_pressed() {
  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_squeeze_released() {
  QNode::left_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_cuff_ok_pressed() {
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_ok_released() {
  QNode::left_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_cuff_grasp_pressed() {
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_grasp_released() {
  QNode::left_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_shoulder_ok_pressed() {
  QNode::left_shoulder_nav.buttons[0] = true;
}

void MainWindow::on_left_shoulder_ok_released() {
  QNode::left_shoulder_nav.buttons[0] = false;
}

void MainWindow::on_left_shoulder_cancel_pressed() {
  QNode::left_shoulder_nav.buttons[1] = true;
}

void MainWindow::on_left_shoulder_cancel_released() {
  QNode::left_shoulder_nav.buttons[1] = false;
}

void MainWindow::on_left_shoulder_show_pressed() {
  QNode::left_shoulder_nav.buttons[2] = true;
}

void MainWindow::on_left_shoulder_show_released() {
  QNode::left_shoulder_nav.buttons[2] = false;
}

void MainWindow::on_left_shoulder_dial_sliderMoved(int position) {
  QNode::left_shoulder_nav.wheel = position;
}

void MainWindow::on_right_shoulder_ok_pressed() {
  QNode::right_shoulder_nav.buttons[0] = true;
}

void MainWindow::on_right_shoulder_ok_released() {
  QNode::right_shoulder_nav.buttons[0] = false;
}

void MainWindow::on_right_shoulder_cancel_pressed() {
  QNode::right_shoulder_nav.buttons[1] = true;
}

void MainWindow::on_right_shoulder_cancel_released() {
  QNode::right_shoulder_nav.buttons[1] = false;
}

void MainWindow::on_right_shoulder_dial_sliderMoved(int position) {
  QNode::right_shoulder_nav.wheel = position;
}

void MainWindow::on_right_shoulder_show_pressed() {
  QNode::right_shoulder_nav.buttons[2] = true;
}

void MainWindow::on_right_shoulder_show_released() {
  QNode::right_shoulder_nav.buttons[2] = false;
}

void MainWindow::on_right_arm_ok_pressed() {
  QNode::right_arm_nav.buttons[0] = true;
}

void MainWindow::on_right_arm_ok_released() {
  QNode::right_arm_nav.buttons[0] = false;
}

void MainWindow::on_right_arm_cancel_pressed() {
  QNode::right_arm_nav.buttons[1] = true;
}

void MainWindow::on_right_arm_cancel_released() {
  QNode::right_arm_nav.buttons[1] = false;
}

void MainWindow::on_right_arm_dial_sliderMoved(int position) {
  QNode::right_arm_nav.wheel = position;
}

void MainWindow::on_right_arm_show_pressed() {
  QNode::right_arm_nav.buttons[2] = true;
}

void MainWindow::on_right_arm_show_released() {
  QNode::right_arm_nav.buttons[2] = false;
}

void MainWindow::on_right_cuff_squeeze_pressed() {
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_squeeze_released() {
  QNode::right_cuff_squeeze.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_right_cuff_ok_pressed() {
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_ok_released() {
  QNode::right_cuff_ok.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_right_cuff_grasp_pressed() {
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_grasp_released() {
  QNode::right_cuff_grasp.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_shoulder_pressed() {
  QNode::left_shoulder.state = baxter_core_msgs::DigitalIOState::PRESSED;
}
void MainWindow::on_left_shoulder_released() {
  QNode::left_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}
void MainWindow::on_right_shoulder_pressed() {
  QNode::right_shoulder.state = baxter_core_msgs::DigitalIOState::PRESSED;
}
void MainWindow::on_right_shoulder_released() {
  QNode::right_shoulder.state = baxter_core_msgs::DigitalIOState::UNPRESSED;
}
}  // namespace baxter_sim_io

