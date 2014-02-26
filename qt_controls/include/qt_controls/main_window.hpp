/**
 * @file /include/qt_controls/main_window.hpp
 *
 * @brief Qt based gui for qt_controls.
 *
 * @date November 2010
 **/
#ifndef qt_controls_MAIN_WINDOW_H
#define qt_controls_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qt_controls {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

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

    void on_left_shoulder_pressed();

    void on_left_shoulder_released();

    void on_right_shoulder_pressed();

    void on_right_shoulder_released();

private:
	//Ui::MainWindowDesign *ui;
Ui::MainWindow *ui;
	QNode qnode;
};

}  // namespace qt_controls

#endif // qt_controls_MAIN_WINDOW_H
