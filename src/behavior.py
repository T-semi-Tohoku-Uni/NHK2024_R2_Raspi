import time
import can
import enum
from enum import Enum
from typing import Dict, List
import hardware_module
from hardware_module import CANList
from NHK2024_Raspi_Library import LogSystem

# 基本的な動作を表すクラス
class BaseAction:
    def __init__(self):
        self.arm = hardware_module.Arm()
        self.fan = hardware_module.VacuumFan()
        pass

    def move(self, v:List):
        gain = [1/16, 1/16, 50]
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


class BehaviorList(Enum):
    INITIALIZING = 0
    INITIALIZED = 4
    START_READY = 8
    ALIVE_AREA1 = 12
    ALIVE_SLOPE12 = 16
    ALIVE_AREA2_OUTER_WALL = 20
    ALIVE_AREA2_WATER_WALL = 24
    ALIVE_APPROACH_SLOPE23 = 27
    ALIVE_SLOPE23 = 28
    ALIVE_AREA3_FIRST_ATTEMPT = 32
    ALIVE_BALL_SEARCH_WIDE = 36
    ALIVE_BALL_SEARCH_NARROW = 40
    ALIVE_BALL_OBTAINIG = 44
    ALIVE_MOVE_TO_SILO = 48
    ALIVE_CHOOSE_SILO = 52
    ALIVE_PUTIN = 56


class Behavior:
    def __init__(self, field: Field, enable_log=True):
        self.base_action = BaseAction()

        self.state = BehaviorList.INITIALIZING
        self.field = field

        self.wall_sensor_state: Dict = {
            "Right rear": False,
            "Right front": False,
            "Front right": False,
            "Front left": False,
            "Left front": False,
            "Left rear": False
        }
        self.posture_state: list[float] = [0, 0, 0, 0]
        self.sensor_state: Dict = {}

        self.max_speed = 1000
        self.position = [0, 0, 0]

        self.can_messages = []

        if enable_log:
            self.log_system = LogSystem()
        else:
            self.log_system = None

    def change_state(self, state: BehaviorList):
        print('Change state from {} to {}'.format(self.state, state))

        if self.log_system is not None:
            self.log_system.write('Change state from {} to {}'.format(self.state, state))
            self.log_system.write('Sensor state: {}'.format(self.sensor_state))
        self.state = state

    def update_sensor_state(self, state: Dict):
        self.sensor_state = state
        self.wall_sensor_state = state['wall_sensor']
        self.posture_state = state['posture']

    def get_state(self):
        return self.state

    def move_along_wall(self, direction: Direction, move_direction: bool = True):
        if direction not in Direction:
            print("Invalid direction")
            return False
        max_speed = 500
        approach_speed = 400
        align_angle_speed = 1

        if direction == Direction.RIGHT:
            
            if self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([approach_speed, max_speed, 0])
            elif self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([approach_speed * 0.1, max_speed, -1 * align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([approach_speed * 0.1, max_speed, align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([approach_speed * 0.1, max_speed, 0])
            else:
                print("Invalid wall sensor state")
                return False
        elif direction == Direction.LEFT.value:
            pass
        elif direction == Direction.FRONT.value:
            pass
        elif direction == Direction.BACK.value:
            pass

    def calculate_position(self):
        pass

    def action(self):
        self.can_messages.clear()
        if self.state == BehaviorList.INITIALIZING:
            self.change_state(BehaviorList.INITIALIZED)
            return self.can_messages

        elif self.state == BehaviorList.INITIALIZED:
            self.change_state(BehaviorList.START_READY)
            return self.can_messages

        #スタート準備OK
        elif self.state == BehaviorList.START_READY:
            self.change_state(BehaviorList.ALIVE_AREA1)
            return self.can_messages

        #エリア１の壁に沿って進む
        elif self.state == BehaviorList.ALIVE_AREA1:
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))
            if self.posture_state[0] < 0:
                self.change_state(self.BehaviorList.ALIVE_SLOPE12)
            return self.can_messages

        #スロープのぼる
        elif self.state == BehaviorList.ALIVE_SLOPE12:
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))

            #坂から抜けだしたら次の状態へ
            if self.posture_state[0] > 0:
                self.can_messages.append(self.change_state(self.BehaviorList.ALIVE_AREA2_OUTER_WALL))
            return self.can_messages

        #エリア２のスロープから水ゾーンの壁への遷移
        elif self.state == BehaviorList.ALIVE_AREA2_OUTER_WALL:
            radius = 2500
            center = 2500
            sign = 1
            if self.field == Field.BLUE:
                sign = 1
            elif self.field == Field.RED:
                sign = -1
            v = [0, self.max_speed, sign * self.max_speed / radius]

            #機体が横向きになったら次の状態へ
            if self.posture_state[0] > center:
                self.change_state(BehaviorList.ALIVE_AREA2_WATER_WALL)

            self.can_messages.append(self.base_action.move(v))
            return self.can_messages

        #水ゾーンの壁に伝って進む
        elif self.state == BehaviorList.ALIVE_AREA2_WATER_WALL:
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))

            #右前のセンサが反応しなくなったら
            if not self.wall_sensor_state[1]:
                self.can_messages.clear()
                #右後ろのセンサが反応しなくなるまでゆっくり進む
                if self.wall_sensor_state[0]:
                    self.can_messages.append(self.base_action.move([0, 300, 0]))

                elif not self.wall_sensor_state[0]:
                    self.change_state(BehaviorList.ALIVE_APPROACH_SLOPE23)

            return self.can_messages

        #スロープに近づく
        elif self.state == BehaviorList.ALIVE_APPROACH_SLOPE23:
            self.can_messages.append(self.base_action.move([0, 300, 0]))


            #右後ろのセンサが反応しなくなったら
            if not self.wall_sensor_state[0]:
                self.change_state(BehaviorList.ALIVE_SLOPE23)
            return self.can_messages


if __name__ == '__main__':
    behavior = Behavior(Field.BLUE, True)
    try:
        while True:
            sensor_state = {
                'wall_sensor': {"Right rear": False, "Right front": False, "Front right": False, "Front left": False, "Left front": False, "Left rear": False},
                'posture': [0, 0, 0, 0]
            }
            behavior.update_sensor_state(sensor_state)

            behavior.action()
            behavior.get_state()

    except KeyboardInterrupt:
        pass
