/**
 * @file /include/qt_controls/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qt_controls_QNODE_HPP_
#define qt_controls_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <baxter_core_msgs/ITBState.h>
#include <baxter_core_msgs/DigitalIOState.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_controls {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	void run();
	static baxter_core_msgs::ITBState left_arm_nav, right_arm_nav, left_shoulder_nav, right_shoulder_nav;
        static baxter_core_msgs::DigitalIOState left_cuff_squeeze, right_cuff_squeeze, left_cuff_ok, 
		right_cuff_ok, left_cuff_grasp, right_cuff_grasp, left_shoulder, right_shoulder;

    void rosShutdown();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher left_itb, right_itb, torso_left_itb, torso_right_itb, left_lower_cuff, 
		right_lower_cuff,left_lower_button, right_lower_button, 
		left_upper_button, right_upper_button, left_shoulder_button, right_shoulder_button;
	ros::Subscriber robot_state;
	
};

}  // namespace qt_controls

#endif /* qt_controls_QNODE_HPP_ */
