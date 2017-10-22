'''
velocity_task.py
'''

import numpy as np

import roombasim.config as cfg

from roombasim.ai import Task, TaskState


class VelocityTask(Task):
    '''
    A task to accelerate/decelerate the drone to a given velocity. The task
    accepts a single 3d vector that defines the target velocty.
    '''

    def __init__(self, target):
        '''
        target - 3d (v_x, v_y, v_z) vector
        '''
        self.target_xy = np.array(target[:2])
        self.target_z = target[2]

        # PID controller contstants
        self.k_xy = cfg.PITTRAS_PID_XY

        self.i_xy = np.array([0, 0], dtype=np.float64)
        self.i_z = 0

    def update(self, delta, elapsed, state_controller, environment):
        # Fetch current odometry
        drone_state = state_controller.query('DroneState', environment)

        if np.linalg.norm(self.target_xy - drone_state['xy_vel']) < \
                cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
            self.complete(TaskState.SUCCESS)
            return

        # Calculate control acceleration vector
        p_xy = self.hold_xy - drone_state['xy_vel']
        d_xy = -drone_state['xy_accel']
        self.i_xy += p_xy * delta
        control_xy = self.k_xy.dot([p_xy, d_xy, self.i_xy])

        # Perform control action
        environment.agent.control(control_xy * delta, 0, self.target_z)
