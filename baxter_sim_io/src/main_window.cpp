/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/baxter_sim_io/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace baxter_sim_io {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv),ui(new Ui::MainWindow)
{
	ui->setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

}

MainWindow::~MainWindow() {}

void MainWindow::on_left_arm_ok_pressed()
{
	QNode::left_arm_nav.buttons[0]=true;
}

void MainWindow::on_left_arm_ok_released()
{
	QNode::left_arm_nav.buttons[0]=false;
}

void MainWindow::on_left_arm_cancel_pressed()
{
	QNode::left_arm_nav.buttons[1]=true;
}

void MainWindow::on_left_arm_cancel_released()
{
	QNode::left_arm_nav.buttons[1]=false;
}

void MainWindow::on_left_arm_dial_sliderMoved(int position)
{
	    QNode::left_arm_nav.wheel=position;
}

void MainWindow::on_left_arm_show_pressed()
{
	QNode::left_arm_nav.buttons[2]=true;
}

void MainWindow::on_left_arm_show_released()
{
	QNode::left_arm_nav.buttons[2]=false;
}	

void MainWindow::on_left_cuff_squeeze_pressed()
{
    QNode::left_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_squeeze_released()
{
    QNode::left_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_cuff_ok_pressed()
{
    QNode::left_cuff_ok.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_ok_released()
{
    QNode::left_cuff_ok.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_cuff_grasp_pressed()
{
    QNode::left_cuff_grasp.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_left_cuff_grasp_released()
{
    QNode::left_cuff_grasp.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_shoulder_ok_pressed()
{
	QNode::left_shoulder_nav.buttons[0]=true;
}

void MainWindow::on_left_shoulder_ok_released()
{
	QNode::left_shoulder_nav.buttons[0]=false;
}

void MainWindow::on_left_shoulder_cancel_pressed()
{
	QNode::left_shoulder_nav.buttons[1]=true;
}

void MainWindow::on_left_shoulder_cancel_released()
{
	QNode::left_shoulder_nav.buttons[1]=false;
}	

void MainWindow::on_left_shoulder_show_pressed()
{
	QNode::left_shoulder_nav.buttons[2]=true;
}

void MainWindow::on_left_shoulder_show_released()
{
	QNode::left_shoulder_nav.buttons[2]=false;
}

void MainWindow::on_left_shoulder_dial_sliderMoved(int position)
{
	QNode::left_shoulder_nav.wheel=position;
}

void MainWindow::on_right_shoulder_ok_pressed()
{
	QNode::right_shoulder_nav.buttons[0]=true;
}

void MainWindow::on_right_shoulder_ok_released()
{
	QNode::right_shoulder_nav.buttons[0]=false;
}

void MainWindow::on_right_shoulder_cancel_pressed()
{
	QNode::right_shoulder_nav.buttons[1]=true;
}

void MainWindow::on_right_shoulder_cancel_released()
{
	QNode::right_shoulder_nav.buttons[1]=false;
}

void MainWindow::on_right_shoulder_dial_sliderMoved(int position)
{
	QNode::right_shoulder_nav.wheel=position;
}

void MainWindow::on_right_shoulder_show_pressed()
{
	QNode::right_shoulder_nav.buttons[2]=true;
}

void MainWindow::on_right_shoulder_show_released()
{
	QNode::right_shoulder_nav.buttons[2]=false;
}

void MainWindow::on_right_arm_ok_pressed()
{
	QNode::right_arm_nav.buttons[0]=true;
}

void MainWindow::on_right_arm_ok_released()
{
	QNode::right_arm_nav.buttons[0]=false;
}

void MainWindow::on_right_arm_cancel_pressed()
{
	QNode::right_arm_nav.buttons[1]=true;
}

void MainWindow::on_right_arm_cancel_released()
{
	QNode::right_arm_nav.buttons[1]=false;
}

void MainWindow::on_right_arm_dial_sliderMoved(int position)
{
	    QNode::right_arm_nav.wheel=position;
}

void MainWindow::on_right_arm_show_pressed()
{
	QNode::right_arm_nav.buttons[2]=true;
}

void MainWindow::on_right_arm_show_released()
{
	QNode::right_arm_nav.buttons[2]=false;
}

void MainWindow::on_right_cuff_squeeze_pressed()
{
    	QNode::right_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_squeeze_released()
{
    	QNode::right_cuff_squeeze.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_right_cuff_ok_pressed()
{
   	 QNode::right_cuff_ok.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_ok_released()
{
    	QNode::right_cuff_ok.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_right_cuff_grasp_pressed()
{
    	QNode::right_cuff_grasp.state=baxter_core_msgs::DigitalIOState::PRESSED;
}

void MainWindow::on_right_cuff_grasp_released()
{
    	QNode::right_cuff_grasp.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}

void MainWindow::on_left_shoulder_pressed()
{
    	QNode::left_shoulder.state=baxter_core_msgs::DigitalIOState::PRESSED;
}
void MainWindow::on_left_shoulder_released()
{
    	QNode::left_shoulder.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}
void MainWindow::on_right_shoulder_pressed()
{
   	QNode::right_shoulder.state=baxter_core_msgs::DigitalIOState::PRESSED;
}
void MainWindow::on_right_shoulder_released()
{
    	QNode::right_shoulder.state=baxter_core_msgs::DigitalIOState::UNPRESSED;
}
}  // namespace baxter_sim_io

