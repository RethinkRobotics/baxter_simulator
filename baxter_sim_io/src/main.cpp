/**
 * @file /src/main.cpp
 *
 * @brief Qt based gui.
 *
 * @date November 2010
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QApplication>
#include "../include/baxter_sim_io/main_window.hpp"
#include "../include/baxter_sim_io/qnode.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    baxter_sim_io::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}

    baxter_core_msgs::ITBState baxter_sim_io::QNode::left_arm_nav, baxter_sim_io::QNode::right_arm_nav,
	baxter_sim_io::QNode::left_shoulder_nav, baxter_sim_io::QNode::right_shoulder_nav;

    baxter_core_msgs::DigitalIOState baxter_sim_io::QNode::left_cuff_squeeze, 
	baxter_sim_io::QNode::right_cuff_squeeze, baxter_sim_io::QNode::left_cuff_ok,
	baxter_sim_io::QNode::right_cuff_ok, baxter_sim_io::QNode::left_cuff_grasp,
	baxter_sim_io::QNode::right_cuff_grasp, baxter_sim_io::QNode::left_shoulder,
	baxter_sim_io::QNode::right_shoulder;
