# SPDX-FileCopyrightText: 2023 Rion Miura <rionmiura39@gmail.com>
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.events import Shutdown

from launch_ros.actions import LifecycleNode
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition

from lifecycle_msgs.msg import Transition


def generate_launch_description():
    ld = LaunchDescription()
    imu_calibration_dir = get_package_share_directory('imu_calibration_data')

    launch_node = GroupAction([
        Node(
            name='imu_calibration_data',
            package='imu_calibration_data',
            executable='imu_calibration_data',
            output='screen')
    ])

    imu_c_data_node = LifecycleNode(
            namespace='',
            name = 'rt_usb_9axisimu_driver',
            package='rt_usb_9axisimu_driver',
            executable='rt_usb_9axisimu_driver',
            output='screen'
    )

    emit_configuring_event_imu_c_data = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(imu_c_data_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    emit_activating_event_imu_c_data = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_action(imu_c_data_node),
            transition_id=Transition.TRANSITION_ACTIVATE,
        )
    )

    emit_shutdown_event = EmitEvent(

        event=Shutdown()
    )

    register_activating_transition_imu_c_data = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_c_data_node,
            goal_state='inactive',
            entities=[
                 emit_activating_event_imu_c_data

            ],
        )
    )

    register_shutting_down_transition_imu_c_data = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=imu_c_data_node,
            goal_state='finalized',
            entities=[
                emit_shutdown_event
            ],
        )
    )

    
    ld = LaunchDescription()
    ld.add_action(launch_node)
    ld.add_action(imu_c_data_node)
    ld.add_action(emit_configuring_event_imu_c_data)
    ld.add_action(emit_activating_event_imu_c_data)
    ld.add_action(register_activating_transition_imu_c_data)
    ld.add_action(register_shutting_down_transition_imu_c_data)


    return ld
