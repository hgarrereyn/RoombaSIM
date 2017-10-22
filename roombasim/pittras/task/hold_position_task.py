'''
hold_position_task.py
'''

import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState


class HoldPositionTaskStates:
    '''
    States for the Hold Position task.
    '''
    init = 0
    holding = 1
    done = 2
    failed = 3


class HoldPositionTask(Task):
    '''
    A task to hold the drone at its current absolute position for some duration
    of time. The task accepts a float64 that specifies such duration in seconds,
    which means holding the current position indefinitely if less than or equal
    to zero.
    '''

    def __init__(self, hold_duration):
        self.hold_duration = hold_duration
        self.state = HoldPositionTaskStates.init

        # Hold position properties
        self.start_time = 0
        self.hold_xy = np.array([0, 0])
        self.hold_z = 0

        # PID controller contstants
        self.k_xy = cfg.PITTRAS_PID_XY
        self.k_z = cfg.PITTRAS_PID_Z

        self.i_xy = np.array([0, 0], dtype=np.float64)
        self.i_z = 0

    def update(self, delta, elapsed, state_controller, environment):
        if (self.state == HoldPositionTaskStates.done or
                self.state == HoldPositionTaskStates.failed):
            return

        # Fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        if self.state == HoldPositionTaskStates.init:
            # TODO(zacyu): Remove this when `elapsed` is changed to represent
            #              the elipsed time since the start of the task.
            self.start_time = elapsed
            self.state = HoldPositionTaskStates.holding
            self.hold_xy = drone_state['xy_pos']
            self.hold_z = drone_state['z_pos']
            return

        if (self.hold_duration > 0) and (elapsed - self.start_time >
                                         self.hold_duration * 1000):
            if (np.linalg.norm(drone_state['xy_pos'] - self.hold_xy) <
                    cfg.PITTRAS_HOLD_POSITION_TOLERANCE and
                    abs(drone_state['z_pos'] - self.hold_z) <
                    cfg.PITTRAS_HOLD_POSITION_TOLERANCE):
                self.complete(TaskState.SUCCESS)
                self.state = HoldPositionTaskStates.done
            else:
                self.complete(TaskState.FAILURE)
                self.state = HoldPositionTaskStates.failed

        # Return to hold position
        p_xy = (self.hold_xy - drone_state['xy_pos'])
        d_xy = -drone_state['xy_vel']
        self.i_xy += p_xy * delta
        control_xy = self.k_xy.dot([p_xy, d_xy, self.i_xy])

        # Rotate the control xy vector counterclockwise about the origin by the
        # current yaw
        yaw = -drone_state['yaw']
        rot_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        adjusted_xy = rot_matrix.dot(control_xy)

        p_z = (self.hold_z - drone_state['z_pos'])
        d_z = -drone_state['z_vel']
        self.i_z += p_z * delta
        control_z = self.k_z.dot([p_z, d_z, self.i_z])

        # Perform control action
        environment.agent.control(adjusted_xy, 0, control_z)
