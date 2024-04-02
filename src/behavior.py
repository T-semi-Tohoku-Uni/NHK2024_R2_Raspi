import time
import can
import enum
from enum import Enum
from typing import Dict, List
import hardware_module
from hardware_module import CANList


# 基本的な動作を表すクラス
class BaseAction:
    def __init__(self):
        self.arm = hardware_module.Arm()
        self.fan = hardware_module.VacuumFan()
        pass

    def move(self, v: [[float], [float], [float]]):
        gain = [0.02, 0.02, 1]
        tx_buffer = bytearray([127, 127, 127])
        for i in range(3):
            tx_buffer[i] = int(v[i] * gain[i] + 127)

        return CANList.ROBOT_VEL.value, tx_buffer


class Direction(Enum):
    RIGHT = 0
    FRONT = 1
    LEFT = 2
    BACK = 3


class Field(Enum):
    BLUE = 0
    RED = 1


class Behavior:
    class BehaviorList(Enum):
        INITIALIZING = 0
        INITIALIZED = 4
        START_READY = 8
        ALIVE_AREA1 = 12
        ALIVE_SLOPE12 = 16
        ALIVE_AREA2_OUTER_WALL = 20
        ALIVE_AREA2_WATER_WALL = 24
        ALIVE_SLOPE23 = 28
        ALIVE_AREA3_FIRST_ATTEMPT = 32
        ALIVE_BALL_SEARCH_WIDE = 36
        ALIVE_BALL_SEARCH_NARROW = 40
        ALIVE_BALL_OBTAINIG = 44
        ALIVE_MOVE_TO_SILO = 48
        ALIVE_CHOOSE_SILO = 52
        ALIVE_PUTIN = 56
    def __init__(self, field=0):
        self.base_action = BaseAction()

        self.state = self.BehaviorList.INITIALIZING
        self.field = field

        self.wall_sensor_state: list[bool] = [False, False, False, False, False, False, False, False]
        self.posture_state: list[float] = {0, 0, 0, 0}

    def change_state(self, state):
        print('Change state from {} to {}'.format(self.state, state))
        self.state = state

    def update_sensor_state(self, state: Dict):
        self.wall_sensor_state = state['wall_sensor']
        # self.posture_state = state['posture']

    def get_state(self):
        return Behavior.state

    def move_along_wall(self, direction: int, move_direction: bool = True):
        if direction > 3:
            print("Invalid direction")
            return False
        max_speed = 500
        apploach_speed = 300
        align_angle_speed = 1

        if direction == Direction.RIGHT.value:
            if self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([apploach_speed, max_speed, 0])
            elif self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([apploach_speed * 0.6, max_speed, -1 * align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([apploach_speed * 0.6, max_speed, align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([apploach_speed * 0.3, max_speed, 0])
            else:
                print("Invalid wall sensor state")
                return False
        elif direction == Direction.LEFT.value:
            pass
        elif direction == Direction.FRONT.value:
            pass
        elif direction == Direction.BACK.value:
            pass

    def action(self):
        if self.state == self.state_list.INITIALIZING:
            self.change_state(self.state_list.INITIALIZIED)
            return
        elif self.state == self.state_list.INITIALIZIED:
            self.change_state(self.state_list.START_READY)
            return
        elif self.state == self.state_list.START_READY:
            self.change_state(self.state_list.ALIVE_AREA1)
            return
        elif self.state == self.state_list.ALIVE_AREA1:
            self.move_along_wall(Direction.RIGHT.value)
            return
        elif self.state == self.state_list.ALIVE_SLOPE12:
            pass

        elif self.state == self.state_list.ALIVE_AREA2_OUTER_WALL:
            pass
