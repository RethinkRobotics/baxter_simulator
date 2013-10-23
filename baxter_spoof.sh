#!/usr/bin/env sh
#spoof sdk enable and gripper state

rostopic pub /robot/state baxter_msgs/AssemblyState "{enabled: true, stopped: false, error: false, estop_button: 0, estop_source: 0}" &

rostopic pub /robot/end_effector/left_gripper/state baxter_core_msgs/EndEffectorState "{timestamp: {secs: 0, nsecs: 0}, id: 1, enabled: 1, calibrated: 1, ready: 1, moving: 0, gripping: 0, missed: 0, error: 0, command: 0, position: 0.0, force: 0.0, state: '{"error_flags" : 0, "op_mode": 2}', command: 'no_op', command_sender: '', command_sequence: 0}" &

rostopic pub /robot/end_effector/right_gripper/state baxter_core_msgs/EndEffectorState "{timestamp: {secs: 0, nsecs: 0}, id: 1, enabled: 1, calibrated: 1, ready: 1, moving: 0, gripping: 0, missed: 0, error: 0, command: 0, position: 0.0, force: 0.0, state: '{"error_flags" : 0, "op_mode": 2}', command: 'no_op', command_sender: '', command_sequence: 0}" &

rostopic pub /robot/end_effector/left_gripper/properties baxter_core_msgs/EndEffectorProperties "{id: 65664, ui_type: 2,  manufacturer: 'test', product: 'test', product: 'test', hardware_rev: 'test', firmware_rev: 'test', firmware_date: 'test', controls_grip: True, senses_grip: True, reverses_grip: True, controls_force: True, senses_force: True, controls_position: True, senses_position: True, properties: ''}" &

rostopic pub /robot/end_effector/right_gripper/properties baxter_core_msgs/EndEffectorProperties "{id: 65664, ui_type: 2,  manufacturer: 'test', product: 'test', product: 'test', hardware_rev: 'test', firmware_rev: 'test', firmware_date: 'test', controls_grip: True, senses_grip: True, reverses_grip: True, controls_force: True, senses_force: True, controls_position: True, senses_position: True, properties: ''}" &
