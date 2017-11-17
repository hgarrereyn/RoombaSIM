
import numpy as np

from roombasim.ai import Task, TaskState

import roombasim.config as cfg
from roombasim import geometry

class GoToRoombaTask(Task):

    def __init__(self, target_roomba, offset_xy):
        '''
        target_roomba - target roomba tag
        offset_xy - float[2] that defines x and y offset from the roomba
        '''
        self.target_roomba = target_roomba
        self.offset_xy = offset_xy

        # PID controller contstants
        self.k_xy = cfg.PITTRAS_PID_XY
        self.k_z = cfg.PITTRAS_PID_Z
        self.k_yaw = cfg.PITTRAS_PID_Z

        self.i_xy = np.array([0, 0], dtype=np.float64)
        self.i_z = 0

        # estimate roomba velocity
        self.last_target_xy = None

    def update(self, delta, elapsed, state_controller, environment):
        # fetch roomba odometry
        target_roombas, _ = state_controller.query('RoombaState', environment)

        # fetch drone odometry
        drone_state = state_controller.query('DroneState', environment)

        if not self.target_roomba in target_roombas:
            self.complete(TaskState.FAILURE, "Roomba not found")
            return

        roomba = target_roombas[self.target_roomba]

        roomba_yaw = roomba['heading']
        rot_matrix = np.array([
            [np.cos(roomba_yaw), -np.sin(roomba_yaw)],
            [np.sin(roomba_yaw), np.cos(roomba_yaw)]
        ])
        adjusted_offset_xy = rot_matrix.dot(self.offset_xy)

        target_xy = np.array(roomba['pos']) + adjusted_offset_xy

        if np.linalg.norm(target_xy - drone_state['xy_pos']) < cfg.PITTRAS_XYZ_TRANSLATION_ACCURACY:
            self.complete(TaskState.SUCCESS)
            return

        if self.last_target_xy is None:
            self.last_target_xy = target_xy

        target_vel_xy = ((target_xy - self.last_target_xy) / delta)

        # xy PID controller
        p_xy = (target_xy - drone_state['xy_pos'])
        d_xy = target_vel_xy - drone_state['xy_vel']
        self.i_xy += p_xy * delta
        control_xy = self.k_xy.dot([p_xy, d_xy, self.i_xy])

        # rotate the control xy vector counterclockwise about
        # the origin by the current yaw
        yaw = -drone_state['yaw']
        rot_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        adjusted_xy = rot_matrix.dot(control_xy)

        # z PID controller
        p_z = (cfg.PITTRAS_TRACK_ROOMBA_HEIGHT - drone_state['z_pos'])
        d_z = - drone_state['z_vel']
        self.i_z += p_z * delta
        control_z = self.k_z.dot([p_z, d_z, self.i_z])

        self.last_target_xy = target_xy

        # perform control action
        environment.agent.control(adjusted_xy, 0, control_z)