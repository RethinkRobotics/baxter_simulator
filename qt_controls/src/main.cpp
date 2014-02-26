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
#include "../include/qt_controls/main_window.hpp"
#include "../include/qt_controls/qnode.hpp"

/*****************************************************************************
** Main
*****************************************************************************/

int main(int argc, char **argv) {

    /*********************
    ** Qt
    **********************/
    QApplication app(argc, argv);
    qt_controls::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    int result = app.exec();

	return result;
}

    baxter_core_msgs::ITBState qt_controls::QNode::left_arm_nav, qt_controls::QNode::right_arm_nav,
	qt_controls::QNode::left_shoulder_nav, qt_controls::QNode::right_shoulder_nav;

    baxter_core_msgs::DigitalIOState qt_controls::QNode::left_cuff_squeeze, 
	qt_controls::QNode::right_cuff_squeeze, qt_controls::QNode::left_cuff_ok,
	qt_controls::QNode::right_cuff_ok, qt_controls::QNode::left_cuff_grasp,
	qt_controls::QNode::right_cuff_grasp, qt_controls::QNode::left_shoulder,
	qt_controls::QNode::right_shoulder;
