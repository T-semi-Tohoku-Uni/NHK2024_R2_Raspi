import time
import can
import enum
from enum import Enum
from typing import Dict, List
import hardware_module
from hardware_module import CANList, Sensors
from NHK2024_Raspi_Library import LogSystem
from math import pi
import math

# 基本的な動作を表すクラス
class BaseAction:
    def __init__(self):
        self.arm = hardware_module.Arm()
        self.fan = hardware_module.VacuumFan()
        pass

    def init_write_can_bus_func(self, write_can_bus):
        self.write_can_bus = write_can_bus
        self.arm.init_write_can_bus_func(write_can_bus)
        self.fan.init_write_can_bus_func(write_can_bus)

    def move(self, v:List):
        gain = [1/16, 1/16, 50]
        tx_buffer = bytearray([127, 127, 127])
        for i in range(3):
            b = int(v[i] * gain[i] + 127)
            if b > 255:
                b = 255
            elif b < 0:
                b = 0
            tx_buffer[i] = int(b)

        self.write_can_bus(CANList.ROBOT_VEL.value, tx_buffer)
        return CANList.ROBOT_VEL.value, tx_buffer
    
    def move_field(self, field_v:list, posture):
        v = [field_v[0] * math.cos(posture), field_v[1] * math.sin(posture), field_v[2]]
        return self.move(v)
    

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
    INITIALIZED = 10
    START_READY = 20
    ALIVE_AREA1 = 30
    ALIVE_SLOPE12 = 40
    ALIVE_AREA2_OUTER_WALL = 50
    ALIVE_AREA2_WATER_WALL = 60
    ALIVE_AREA2_ON_WATER_WALL = 62
    ALIVE_APPROACH_SLOPE23_ROTATE = 68
    ALIVE_APPROACH_SLOPE23 = 70
    ALIVE_SLOPE23 = 80
    ALIVE_AREA3_FIRST_ATTEMPT = 90
    ALIVE_AREA3_FOLLOW_STRAGE_CENTERLINE = 95
    ALIVE_BALL_SEARCH_WIDE = 100
    ALIVE_BALL_SEARCH_NARROW = 110
    ALIVE_BALL_OBTAINIG = 120
    ALIVE_BALL_PICKUP_WAITING = 121
    ALIVE_FIND_SILOLINE = 130
    ALIVE_FOLLOW_SILOLINE = 140
    ALIVE_MOVE_TO_SILO = 150
    ALIVE_CHOOSE_SILO = 160
    ALIVE_ALIGN_SILOZONE = 169
    ALIVE_PUTIN = 170
    ALIVE_PUTIN_WAIT = 171
    ALIVE_MOVE_TO_STORAGE = 180
    FINISH = 1000


class Behavior:
    def __init__(self, field: Field, 
                 center_obtainable_area, 
                 enable_log=True, 
                 start_state: BehaviorList = BehaviorList.INITIALIZING, 
                 finish_state: BehaviorList = BehaviorList.FINISH
                 ):
        
        self.base_action = BaseAction()

        
        self.field = field
        self.state = start_state
        self.finish_state = finish_state


        self.wall_sensor_state: Dict = {
            "Right rear": False,
            "Right front": False,
            "Front right": False,
            "Front left": False,
            "Left front": False,
            "Left rear": False
        }

        self.sensor_state: Dict = {}
        self.is_on_slope = False
        self.posture = 0
        self.robot_vel = [0, 0, 0]
        self.ball_camera = ()
        self.line_camera = ()

        self.center_obtainable_area = center_obtainable_area

        self.max_speed = 300
        self.position = [0, 0, 0]
        self.stored_balls = 0

        self.can_messages = []
        self.log_system = None

    def init_log_system(self, log_system):
        self.log_system = log_system

    def init_write_can_bus(self, write_can_bus):
        self.base_action.init_write_can_bus_func(write_can_bus)

    def change_state(self, state: BehaviorList):
        print('Change state from {} to {}'.format(self.state, state))

        if self.log_system is not None:
            self.log_system.write('Change state from {} to {}'.format(self.state, state))
            self.log_system.write('Sensor state: {}'.format(self.sensor_state))
        self.state = state

    def update_sensor_state(self, state: Dict):
        self.sensor_state = state
        self.wall_sensor_state = state[Sensors.WALL_SENSOR]
        self.is_on_slope = state[Sensors.IS_ON_SLOPE]
        self.ball_camera = state[Sensors.BALL_CAMERA]
        self.line_camera = state[Sensors.LINE_CAMERA]
        self.posture_state = state[Sensors.POSTURE]

    def get_state(self):
        return self.state

    def follow_object(self, object_pos_error):
        gain = [2, 2, 0]
        v = [0, 0, 0]
        for i in range(3):
            v[i] = gain[i] * object_pos_error[i]
        return self.base_action.move(v)


    def move_along_wall(self, direction: Direction, move_direction: bool = True, approach_speed = 300):
        if direction not in Direction:
            print("Invalid direction")
            return False
        
        virtual_thrust_speed = 100
        align_angle_speed = 0.1

        if direction == Direction.RIGHT:
            
            if self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([approach_speed, self.max_speed, 0])
            elif self.wall_sensor_state['Right front'] == False and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([virtual_thrust_speed, self.max_speed, -1 * align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == False:
                return self.base_action.move([virtual_thrust_speed, self.max_speed, align_angle_speed])
            elif self.wall_sensor_state['Right front'] == True and self.wall_sensor_state['Right rear'] == True:
                return self.base_action.move([virtual_thrust_speed, self.max_speed, 0])
            else:
                print("Invalid wall sensor state")
                return False
        elif direction == Direction.LEFT:
            pass
        elif direction == Direction.FRONT:
            pass
        elif direction == Direction.BACK:
            pass

    def shutdown(self):
        self.log_system.write('Call shutdown function')
        print('Call shutdown function')
        self.change_state(BehaviorList.FINISH)
        self.base_action.fan.off()
        self.base_action.arm.up()
        self.base_action.move([0, 0, 0])
        exit()

    def calculate_position(self):
        pass

    def follow_object(self, pos_error:List, gain = (1, 1, 0.5)):
        v = [0, 0, 0]

        # print(pos)

        for i in range(3):
            v[i] = gain[i] * pos_error[i]

        return self.base_action.move(v)

    def action(self):
        if self.state == self.finish_state:
            self.state = BehaviorList.FINISH

        self.can_messages.clear()
        if self.state == BehaviorList.INITIALIZING:
            self.can_messages.append(self.base_action.fan.off())
            self.can_messages.append(self.base_action.arm.up())
            self.change_state(BehaviorList.INITIALIZED)

        elif self.state == BehaviorList.INITIALIZED:
            time.sleep(1)
            self.change_state(BehaviorList.START_READY)

        #スタート準備OK
        elif self.state == BehaviorList.START_READY:
            self.change_state(BehaviorList.ALIVE_AREA1)
            #self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)

        #エリア１の壁に沿って進む
        elif self.state == BehaviorList.ALIVE_AREA1:
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))
            if self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_SLOPE12)

        #スロープのぼる
        elif self.state == BehaviorList.ALIVE_SLOPE12:
            if self.field == Field.BLUE:
                self.can_messages.append(self.base_action.move([-30, self.max_speed, 0]))
            elif self.field == Field.RED:
                self.can_messages.append(self.base_action.move([30, self.max_speed, 0]))

            #坂から抜けだしたら次の状態へ
            if not self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_AREA2_OUTER_WALL)

        #エリア２のスロープから水ゾーンの壁への遷移
        #半径2500mmの円を描いて方向転換
        elif self.state == BehaviorList.ALIVE_AREA2_OUTER_WALL:
            radius = 2500
            sign = 1
            if self.field == Field.BLUE:
                sign = 1
            elif self.field == Field.RED:
                sign = -1
            v = [0, self.max_speed, sign * self.max_speed / radius]

            #print(self.posture)
            #機体が横向きになったら次の状態へ
            if self.posture > pi/2:
                self.change_state(BehaviorList.ALIVE_AREA2_WATER_WALL)

            self.can_messages.append(self.base_action.move(v))

        #水ゾーンの壁に伝って進む
        elif self.state == BehaviorList.ALIVE_AREA2_WATER_WALL:
            self.max_speed = 0
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT, approach_speed=300))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))

            if self.wall_sensor_state['Right front']and self.wall_sensor_state['Right rear'] == True:
                time.sleep(0.35)
                self.change_state(BehaviorList.ALIVE_AREA2_ON_WATER_WALL)

        elif self.state == BehaviorList.ALIVE_AREA2_ON_WATER_WALL:
            self.max_speed = 500
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))

            if not self.wall_sensor_state['Right front']:
                time.sleep(0.3)
                self.change_state(BehaviorList.ALIVE_APPROACH_SLOPE23_ROTATE)

        elif self.state == BehaviorList.ALIVE_APPROACH_SLOPE23_ROTATE:
            self.max_speed = 300
            radius = 300
            sign = -1
            if self.field == Field.BLUE:
                sign = -1
            elif self.field == Field.RED:
                sign = 1
            v = [0, self.max_speed, sign * self.max_speed / radius]

            #print(self.posture)
            #機体が横向きになったら次の状態へ
            if self.posture < 0:
                self.change_state(BehaviorList.ALIVE_APPROACH_SLOPE23)

            self.can_messages.append(self.base_action.move(v))

        #スロープに近づく
        elif self.state == BehaviorList.ALIVE_APPROACH_SLOPE23:
            self.can_messages.append(self.base_action.move([0, 300, 0]))

            # 坂検出
            if self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_SLOPE23)
        
        elif self.state == BehaviorList.ALIVE_SLOPE23:
            self.can_messages.append(self.base_action.move([0, self.max_speed, 0]))

            #スロープ検出した後，平面検出するまで進む
            if not self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_AREA3_FIRST_ATTEMPT)
        
        #半径2000の円を描きながら適当に真ん中あたりに行く
        elif self.state == BehaviorList.ALIVE_AREA3_FIRST_ATTEMPT:
            radius = 2000
            self.max_speed = 500
            sign = -1
            if self.field == Field.BLUE:
                sign = -1

            if self.field == Field.RED:
                sign = 1
            self.can_messages.append(self.base_action.move([0, self.max_speed, sign * self.max_speed/radius]))

            num, x, y, z, is_obtainable = self.ball_camera

            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)
            
            elif self.line_camera[0] and self.posture < -11 / 24 * pi:
                self.change_state(BehaviorList.ALIVE_AREA3_FOLLOW_STRAGE_CENTERLINE)

            elif self.posture < -pi/2:
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_WIDE)

        # ストレージエリアのライン追従
        elif self.state == BehaviorList.ALIVE_AREA3_FOLLOW_STRAGE_CENTERLINE:
            pos = self.line_camera[3], self.max_speed/3, -pi/2 - self.posture
            self.can_messages.append(self.follow_object(pos))
            
            num, x, y, z, is_obtainable = self.ball_camera
            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)
        
        # ボール探し
        elif self.state == BehaviorList.ALIVE_BALL_SEARCH_WIDE:
            self.can_messages.append(self.base_action.move([0, 0, 0.3]))
            num, x, y, z, is_obtainable = self.ball_camera

            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)

        # ボール回収
        elif self.state == BehaviorList.ALIVE_BALL_OBTAINIG:
            num, x, y, z, is_obtainable = self.ball_camera
            if num > 0:
                pos = x - self.center_obtainable_area[0], y - self.center_obtainable_area[1], 0
                msg = self.follow_object(pos)

                self.can_messages.append(msg)
            
            elif num == 0:
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_WIDE)

            if is_obtainable:
                self.can_messages.append(self.base_action.arm.down())
                self.can_messages.append(self.base_action.fan.on())
                # for c in self.can_messages:
                #     self.write_can_bus(c)
                self.change_state(BehaviorList.ALIVE_BALL_PICKUP_WAITING)
        
        elif self.state == BehaviorList.ALIVE_BALL_PICKUP_WAITING:
            time.sleep(1)
            self.change_state(BehaviorList.ALIVE_BALL_SEARCH_WIDE)
        
        # サイロエリアのラインを探す
        elif self.state == BehaviorList.ALIVE_FIND_SILOLINE:
            if self.line_camera[0] and (self.posture > pi * 1/4 or self.posture < pi * 3 / 4):
                self.change_state(BehaviorList.ALIVE_FOLLOW_SILOLINE)

            v = [-self.max_speed, 0, pi/2 - self.posture]
            self.can_messages.append(self.base_action.move_field(v))

            if self.wall_sensor_state['Front right'] or self.wall_sensor_state['Front left']:
                self.change_state(BehaviorList.FINISH)
                
            
        elif self.state == BehaviorList.ALIVE_FOLLOW_SILOLINE:
            vertical, right, left, error = self.line_camera
            # 縦ラインに追従
            if vertical:
                self.can_messages.append(self.follow_object([error, 500, 0], gain=(0.5, 1, 0)))
                if self.wall_sensor_state['Front right'] or self.wall_sensor_state['Front left']:
                    self.change_state(BehaviorList.ALIVE_AILGN_SILOZONE)
            
            # ラインがなかったら探しに行く
            else :
                self.change_state(BehaviorList.ALIVE_FIND_SILOLINE)

        elif self.state == BehaviorList.ALIVE_ALIGN_SILOZONE:
            self.can_messages.append(self.move_along_wall(Direction.FRONT))
            if self.wall_sensor_state['Front right'] and self.wall_sensor_state['Front left']:
                self.change_state(BehaviorList.ALIVE_PUTIN)

        elif self.state == BehaviorList.ALIVE_PUTIN:
            self.can_messages.append(self.base_action.fan.off())
            self.stored_balls = self.stored_balls + 1
            self.change_state(BehaviorList.ALIVE_PUTIN_WAIT)
            if self.stored_balls > 6:
                self.change_state(BehaviorList.FINISH)

        elif self.state == BehaviorList.ALIVE_PUTIN_WAIT:
            time.sleep(1)
            self.change_state(BehaviorList.ALIVE_MOVE_TO_STORAGE)
        
        elif self.state == BehaviorList.ALIVE_MOVE_TO_STORAGE:
            rot = - pi / 2 - self.posture
            self.base_action.move_field([self.max_speed, 0, rot], self.posture)
            num, x, y, z, is_obtainable = self.ball_camera

            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)

            if self.posture < - pi / 2 :
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_WIDE)

        
        elif self.state == BehaviorList.FINISH:
            self.shutdown()
                
        return self.can_messages


if __name__ == '__main__':
    behavior = Behavior(Field.BLUE, True)
    try:
        while True:
            sensor_state = {
                'wall_sensor': {"Right rear": False, "Right front": False, "Front right": False, "Front left": False, "Left front": False, "Left rear": False},
                'posture': [0, 0, 0, 0],
                'ball_camera':(0, 1, 0, 600, 0, False)
            }
            behavior.update_sensor_state(sensor_state)

            behavior.action()
            behavior.get_state()

    except KeyboardInterrupt:
        pass
