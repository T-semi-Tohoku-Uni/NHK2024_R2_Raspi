import time
import can
import enum
from enum import Enum
from typing import Dict, List
import hardware_module
from hardware_module import CANList, Sensors, Field
from NHK2024_Raspi_Library import LogSystem
from math import pi
import math
import numpy as np
from silo_decision import ScoreingSilos

# 基本的な動作を表すクラス
class BaseAction:
    def __init__(self):
        self.arm = hardware_module.Arm()
        self.fan = hardware_module.VacuumFan()
        self.cam_arm = hardware_module.CamArm()
        pass

    def init_write_can_bus_func(self, write_can_bus):
        self.write_can_bus = write_can_bus
        self.arm.init_write_can_bus_func(write_can_bus)
        self.fan.init_write_can_bus_func(write_can_bus)
        self.cam_arm.init_write_can_bus_func(write_can_bus)

    def move(self, v:List, is_field = False):
        gain = [1/16, 1/16, 50]
        lim = [60, 80, 100]
            
        tx_buffer = bytearray([127, 127, 127, 0])
        
        if is_field:
            tx_buffer[3] = 1
        else :
            tx_buffer[3] = 0

        for i in range(3):
            b = int(v[i] * gain[i] + 127)
            if b > 127 + lim[i]:
                b = 127 + lim[i]
            elif b < 127 - lim[i]:
                b = 127 - lim[i]
            tx_buffer[i] = int(b)

        self.write_can_bus(CANList.ROBOT_VEL.value, tx_buffer)
        return CANList.ROBOT_VEL.value, tx_buffer
    

class Direction(Enum):
    RIGHT = 0
    FRONT = 1
    LEFT = 2
    BACK = 3


class BehaviorList(Enum):
    INITIALIZING = 0
    INITIALIZED = 10
    START_READY = 20
    ALIVE_AREA1 = 30
    ALIVE_SLOPE12 = 40
    ALIVE_AREA2_OUTER_WALL = 50
    ALIVE_AREA2_WATER_WALL = 60
    RETRY_AREA2 = 65
    ALIVE_AREA2_ON_WATER_WALL = 62
    ALIVE_APPROACH_SLOPE23_ROTATE = 68
    ALIVE_APPROACH_SLOPE23 = 70
    ALIVE_SLOPE23 = 80
    ALIVE_AREA3_FIRST_ATTEMPT = 90
    ALIVE_AREA3_FOLLOW_STRAGE_CENTERLINE = 95
    ALIVE_BALL_SEARCH_CCW = 100
    ALIVE_BALL_SEARCH_CW = 110
    ALIVE_BALL_OBTAINIG = 120
    ALIVE_BALL_OBTAINIG_CW = 122
    ALIVE_BALL_PICKUP_WAITING = 121
    ALIVE_FIND_SILO = 130
    ALIVE_ANOTHER_SILO_CW = 132
    ALIVE_ANOTHER_SILO_CCW = 134
    ALIVE_CHECK_SILO = 140
    ALIVE_MOVE_TO_SILO = 150
    ALIVE_CHOOSE_SILO = 160
    ALIVE_ALIGN_SILOZONE = 169
    ALIVE_PUTIN = 170
    ALIVE_PUTIN_WAIT = 171
    ALIVE_MOVE_TO_STORAGE = 180
    ALIVE_ARRIVE_AT_STORAGE = 190
    FINISH = 1000


class Behavior:
    def __init__(self, field: Field, 
                 center_obtainable_area, 
                 enable_log=True, 
                 start_state: BehaviorList = BehaviorList.ALIVE_AREA1, 
                 finish_state: BehaviorList = BehaviorList.FINISH
                 ):
        
        self.base_action = BaseAction()
        
        self.field = field
        self.state = BehaviorList.INITIALIZING
        self.start_state = start_state
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
        self.ui_buttons = 0
        self.posture = 0
        self.robot_vel = [0, 0, 0]
        self.ball_camera = ()
        self.line_camera = ()
        self.silo_camera = ()
        self.center_obtainable_area = center_obtainable_area

        self.obtainable_counter = 0

        self.max_speed = 800
        self.position = [0, 0, 0]
        self.stored_balls = 0

        self.can_messages = []
        self.log_system = None        
        self.main_log_file_name = 'behavior.log'

    def init_log_system(self, log_system):
        self.log_system = log_system
        self.log_system.create_new_log(self.main_log_file_name)

    def init_write_can_bus(self, write_can_bus):
        self.base_action.init_write_can_bus_func(write_can_bus)

    def change_state(self, state: BehaviorList):
        print('Change state from {} to {}'.format(self.state, state))

        if self.log_system is not None:
            self.log_system.write('Change state from {} to {}'.format(self.state, state), self.main_log_file_name)
            self.log_system.write('Sensor state: {}'.format(self.sensor_state), self.main_log_file_name)
            self.log_system.write('Change state from {} to {}'.format(self.state, state))
            self.log_system.write('Sensor state: {}'.format(self.sensor_state))
        self.state = state

    def update_sensor_state(self, state: Dict):
        self.sensor_state = state
        self.wall_sensor_state = state[Sensors.WALL_SENSOR]
        self.is_on_slope = state[Sensors.IS_ON_SLOPE]
        self.ball_camera = state[Sensors.BALL_CAMERA]
        self.line_camera = state[Sensors.LINE_CAMERA]
        self.posture = state[Sensors.POSTURE]
        self.robot_vel = state[Sensors.ROBOT_VEL]
        self.ui_buttons = state[Sensors.UI_BUTTONS]
        self.silo_camera = state[Sensors.SILO_CAMERA]
        

        self.log_system.write('Update sensor state: {}'.format(self.sensor_state), self.main_log_file_name)

    def get_state(self):
        return self.state

    def follow_object(self, object_pos_error):
        gain = [2, 2, 0.3]
        v = [0, 0, 0]
        for i in range(3):
            v[i] = gain[i] * object_pos_error[i]
        return self.base_action.move(v, False)


    def move_along_wall(self, direction: Direction, move_direction: bool = True, approach_speed = 300):
        if direction not in Direction:
            print("Invalid direction")
            return False
        
        virtual_thrust_speed = 50
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
            if self.wall_sensor_state['Left front'] == False and self.wall_sensor_state['Left rear'] == False:
                return self.base_action.move([-approach_speed, self.max_speed, 0])
            elif self.wall_sensor_state['Left front'] == False and self.wall_sensor_state['Left rear'] == True:
                return self.base_action.move([-virtual_thrust_speed, self.max_speed, -1 * align_angle_speed])
            elif self.wall_sensor_state['Left front'] == True and self.wall_sensor_state['Left rear'] == False:
                return self.base_action.move([-virtual_thrust_speed, self.max_speed, align_angle_speed])
            elif self.wall_sensor_state['Left front'] == True and self.wall_sensor_state['Left rear'] == True:
                return self.base_action.move([-virtual_thrust_speed, self.max_speed, 0])
            else:
                print("Invalid wall sensor state")
                return False
            pass
        elif direction == Direction.FRONT:
            if self.wall_sensor_state['Front right'] == False and self.wall_sensor_state['Front left'] == False:
                return self.base_action.move([0, approach_speed, 0])
            elif self.wall_sensor_state['Front right'] == False and self.wall_sensor_state['Front left']:
                return self.base_action.move([0, virtual_thrust_speed, align_angle_speed])
            elif self.wall_sensor_state['Front right'] and self.wall_sensor_state['Front left'] == False:
                return self.base_action.move([0, virtual_thrust_speed, -align_angle_speed])
            elif self.wall_sensor_state['Front right'] and self.wall_sensor_state['Front left']:
                return self.base_action.move([0, virtual_thrust_speed, 0])
        
        elif direction == Direction.BACK:
            pass

    def shutdown(self):
        self.log_system.write('Call shutdown function')
        print('Call shutdown function')
        if self.get_state() != BehaviorList.FINISH:
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
    
    def is_heading_up(self, margin):
        if abs(self.posture) < margin:
            return True
        else:
            return False
        
    def is_heading_down(self, margin):
        if abs(self.posture) > (pi - margin):
            return True
        else:
            return False
        
    def is_heading_in(self, margin):
        if abs(self.posture - (pi / 2 * self.field.value)) < margin:
            return True
        else:
            return False

    def is_heading_out(self, margin):
        if abs(self.posture - (-pi / 2 * self.field.value)) < margin:
            return True
        else:   
            return False
        
    def action(self):
        if self.state == self.finish_state or (self.ui_buttons == 0 and self.get_state().value>BehaviorList.ALIVE_AREA1.value):
            self.state = BehaviorList.FINISH
        if self.ui_buttons == 0 and self.state.value > BehaviorList.START_READY.value:
            self.change_state(BehaviorList.START_READY)
            return
        
        self.can_messages.clear()
        if self.state == BehaviorList.INITIALIZING:
            self.base_action.fan.off()
            self.base_action.arm.up()
            self.base_action.cam_arm.down()
            self.max_speed = 800
            self.change_state(BehaviorList.INITIALIZED)

        elif self.state == BehaviorList.INITIALIZED:
            self.base_action.cam_arm.up()
            time.sleep(1)
            self.base_action.cam_arm.up()
            self.change_state(BehaviorList.START_READY)

        #スタート準備OK
        elif self.state == BehaviorList.START_READY:
            self.base_action.move([0, 0, 0])
            if self.ui_buttons == 0:
                return
            if self.ui_buttons == 1:
                self.base_action.cam_arm.up()
                self.change_state(self.start_state)
            if self.ui_buttons == 2:
                self.max_speed = 600
                self.change_state(BehaviorList.ALIVE_AREA1)
            if self.ui_buttons == 3:
                self.max_speed = 600
                self.change_state(BehaviorList.RETRY_AREA2)

        #エリア１の壁に沿って進む
        elif self.state == BehaviorList.ALIVE_AREA1:
            if self.field == Field.BLUE:
                self.move_along_wall(Direction.RIGHT)
            elif self.field == Field.RED:
                self.move_along_wall(Direction.LEFT)
            if self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_SLOPE12)

        #スロープのぼる
        elif self.state == BehaviorList.ALIVE_SLOPE12:
            if self.field == Field.BLUE:
                self.base_action.move([-20, self.max_speed, 0])
            elif self.field == Field.RED:
                self.base_action.move([20, self.max_speed, 0])

            #坂から抜けだしたら次の状態へ
            if not self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_AREA2_OUTER_WALL)

        #エリア２のスロープから水ゾーンの壁への遷移
        #半径2500mmの円を描いて方向転換
        elif self.state == BehaviorList.ALIVE_AREA2_OUTER_WALL:
            radius = 2300
            sign = 1
            if self.field == Field.BLUE:
                sign = 1
            elif self.field == Field.RED:
                sign = -1
            v = [0, self.max_speed, sign * self.max_speed / radius]

            #print(self.posture)
            #機体が横向きになったら次の状態へ
            if self.is_heading_in(pi/12):
                self.change_state(BehaviorList.ALIVE_AREA2_WATER_WALL)

            self.can_messages.append(self.base_action.move(v))

        #水ゾーンの壁に伝って進む
        elif self.state == BehaviorList.ALIVE_AREA2_WATER_WALL:
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT, approach_speed=250))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT, approach_speed=250))

            if ((self.wall_sensor_state['Right front'] or self.wall_sensor_state['Right rear']) and self.field == Field.BLUE) or ((self.wall_sensor_state['Left front'] or self.wall_sensor_state['Left rear'] == True) and self.field == Field.RED):
                time.sleep(0.4)
                self.change_state(BehaviorList.ALIVE_AREA2_ON_WATER_WALL)
            
        elif self.state == BehaviorList.RETRY_AREA2:
            vx = self.field.value * 400
            self.base_action.move([vx, 600, 0])
            if ((self.wall_sensor_state['Right front'] or self.wall_sensor_state['Right rear']) and self.field == Field.BLUE) or ((self.wall_sensor_state['Left front'] or self.wall_sensor_state['Left rear'] == True) and self.field == Field.RED):
                time.sleep(0.4)
                self.change_state(BehaviorList.ALIVE_AREA2_ON_WATER_WALL)

        elif self.state == BehaviorList.ALIVE_AREA2_ON_WATER_WALL:
            self.max_speed = 800
            if self.field == Field.BLUE:
                self.can_messages.append(self.move_along_wall(Direction.RIGHT))
            elif self.field == Field.RED:
                self.can_messages.append(self.move_along_wall(Direction.LEFT))

            if (not self.wall_sensor_state['Right front'] and not self.wall_sensor_state['Right rear'] == True and self.field == Field.BLUE) or (not self.wall_sensor_state['Left front'] and not self.wall_sensor_state['Left rear'] == True and self.field == Field.RED):
                self.change_state(BehaviorList.ALIVE_APPROACH_SLOPE23_ROTATE)

        elif self.state == BehaviorList.ALIVE_APPROACH_SLOPE23_ROTATE:
            self.max_speed = 300
            radius = 200
            sign = -1
            if self.field == Field.BLUE:
                sign = -1
            elif self.field == Field.RED:
                sign = 1
            v = [0, self.max_speed, sign * self.max_speed / radius]

            #機体が横向きになったら次の状態へ
            if self.is_heading_up(0.1*pi):
                self.change_state(BehaviorList.ALIVE_APPROACH_SLOPE23)

            self.base_action.move(v)

        #スロープに近づく
        elif self.state == BehaviorList.ALIVE_APPROACH_SLOPE23:
            self.base_action.move([0, 1500, -self.posture])

            # 坂検出
            if self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_SLOPE23)
        
        elif self.state == BehaviorList.ALIVE_SLOPE23:
            self.can_messages.append(self.base_action.move([0, 1500, -self.posture]))

            #スロープ検出した後，平面検出するまで進む
            if not self.is_on_slope:
                self.change_state(BehaviorList.ALIVE_AREA3_FIRST_ATTEMPT)
        
        #半径2000の円を描きながら適当に真ん中あたりに行く
        elif self.state == BehaviorList.ALIVE_AREA3_FIRST_ATTEMPT:
            self.base_action.arm.up()
            radius = 2000
            if self.field == Field.RED:
                radius = 1500

            self.max_speed = 1500
            sign = -1
            if self.field == Field.BLUE:
                sign = -1

            if self.field == Field.RED:
                sign = 1
            self.base_action.move([0, self.max_speed, sign * self.max_speed/radius])

            num, x, y, z, is_obtainable = self.ball_camera

            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)
            
            # elif self.line_camera[0] and self.posture < -11 / 24 * pi:
            #     self.change_state(BehaviorList.ALIVE_AREA3_FOLLOW_STRAGE_CENTERLINE)

            if self.is_heading_out(0.1*pi):
                self.base_action.move([x * 1.2 - 200, 600, (-pi/2 - self.posture) * 0.3])
                # time.sleep(4)
                # self.ball_camera = (0, 0, 0, 0, 0, False)
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CW)
        
        # ボール探し
        elif self.state == BehaviorList.ALIVE_BALL_SEARCH_CCW:
            sign = 1
            if self.field == Field.BLUE:
                sign = 1

            if self.field == Field.RED:
                sign = -1

            vx = 0
            if self.wall_sensor_state['Left rear'] or self.wall_sensor_state['Left front']:
                vx += 400
            if self.wall_sensor_state['Right rear'] or self.wall_sensor_state['Right front']:
                vx -= 400
            
            self.base_action.move([vx, 0, sign*0.8])
            num, x, y, z, is_obtainable = self.ball_camera

            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG)
            elif self.is_heading_up(0.5):
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CW)

        elif self.state == BehaviorList.ALIVE_BALL_SEARCH_CW:
            num, x, y, z, is_obtainable = self.ball_camera

            vx = 0
            if self.wall_sensor_state['Left rear'] or self.wall_sensor_state['Left front']:
                vx += 400
            if self.wall_sensor_state['Right rear'] or self.wall_sensor_state['Right front']:
                vx -= 400
            if num > 0:
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG_CW)
            elif self.is_heading_down(0.5):
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CCW)
            self.base_action.move([vx, 0, -self.field.value*0.8])
 
        # ボール回収
        elif self.state == BehaviorList.ALIVE_BALL_OBTAINIG:
            num, x, y, z, is_obtainable = self.ball_camera
            if num > 0:
                target = 0
                if self.wall_sensor_state['Right front'] or self.wall_sensor_state['Right rear']:
                    target = pi/8
                elif self.wall_sensor_state['Left front'] or self.wall_sensor_state['Left rear']:
                    target = -pi/8
                pos = x - self.center_obtainable_area[0], y - self.center_obtainable_area[1], -self.field.value * pi/2 + target - self.posture
                self.follow_object(pos, gain=(1.4, 1.8, 0))
            
            elif num == 0:
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CCW)

            if is_obtainable:
                self.obtainable_counter += 1

            if not is_obtainable:
                self.obtainable_counter = 0

            if self.obtainable_counter > 10:
                self.base_action.move([0, 0, 0])
                self.base_action.fan.on()
                self.base_action.move([0, 0, 0])
                self.base_action.arm.down()
                self.obtainable_counter = 0
                
                self.change_state(BehaviorList.ALIVE_BALL_PICKUP_WAITING)

        elif self.state == BehaviorList.ALIVE_BALL_OBTAINIG_CW:
            num, x, y, z, is_obtainable = self.ball_camera
            if num > 0:
                pos = x - self.center_obtainable_area[0], y - self.center_obtainable_area[1], -self.field.value * pi/2 - self.posture
                self.follow_object(pos, gain=(1.4, 1.8, 0))
            
            elif num == 0:
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CW)

            if is_obtainable:
                self.obtainable_counter += 1

            if not is_obtainable:
                self.obtainable_counter = 0

            if self.obtainable_counter > 10:
                self.base_action.move([0, 0, 0])
                self.base_action.fan.on()                
                self.base_action.move([0, 0, 0])
                self.base_action.arm.down()
                self.obtainable_counter = 0
                
                self.change_state(BehaviorList.ALIVE_BALL_PICKUP_WAITING)

        
        elif self.state == BehaviorList.ALIVE_BALL_PICKUP_WAITING:
            self.base_action.fan.on()
            time.sleep(1.5)
            self.base_action.arm.up()
            time.sleep(1.5)
            self.change_state(BehaviorList.ALIVE_MOVE_TO_SILO)
        
        # サイロエリアに向かう
        elif self.state == BehaviorList.ALIVE_MOVE_TO_SILO:
            # not obtained:
            # num, x, y, z, is_obtainable = self.ball_camera
            # if num == 0 or (num > 0 and z > 500):
            #     self.base_action.move([0, -self.field.value * 1500 , 0])
            #     return
            
            # 90度の方を向いたら
            if self.is_heading_in(pi * 0.2):
                self.base_action.fan.on()
                self.change_state(BehaviorList.ALIVE_FIND_SILO)
            
            # num, x, y, z, is_obtainable = self.ball_camera
            # if num == 0 or (num >= 1 and z > 500):
            #     self.obtainable_counter += 1
            # else:
            #     self.obtainable_counter = 0
            # if self.obtainable_counter > 30:
            #     self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CW)

            gain = 8
            v = [-self.field.value * 1500, 0, (self.field.value * pi/2 - self.posture) * gain]
            self.base_action.move(v, is_field=True)
                
        # サイロを見つける
        elif self.state == BehaviorList.ALIVE_FIND_SILO:
            self.base_action.fan.on()
            rad = self.field.value * pi / 2 - self.posture
            
            if self.silo_camera:
                scoreing_silos = ScoreingSilos(self.silo_camera)
                sorted_silos = scoreing_silos.sort_by_score()
                x, y, _ = sorted_silos[0].pos
                # print(x, y, _)
                self.follow_object([x, y-150, rad], (0.7, 0.8, 0.5))
                if (sorted_silos[0].get_my_team_ball_cnt() + sorted_silos[0].get_opponent_team_cnt())>=3:
                    x_xum = 0
                    # new silo are might be in the direction where sum of all x coordinate
                    for silo in sorted_silos:
                        x, y, _ = silo.pos
                        x_sum = x
                    
                    if x_sum >= 0:
                        self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CCW)

                    if x_sum < 0:
                        self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CW)
                
                if y < 1000:
                    self.change_state(BehaviorList.ALIVE_CHECK_SILO)

            else:
                self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CW)

        elif self.state == BehaviorList.ALIVE_ANOTHER_SILO_CCW:
            self.base_action.fan.on()
            scoreing_silos = ScoreingSilos(self.silo_camera)
            sorted_silos = scoreing_silos.sort_by_score()
            vx = 0

            if sorted_silos:
                __, y, _ = sorted_silos[0].pos
                if (sorted_silos[0].get_my_team_ball_cnt() + sorted_silos[0].get_opponent_team_cnt()) < 3:
                    self.change_state(BehaviorList.ALIVE_FIND_SILO)
                    pass

                elif self.is_heading_down(1):
                    self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CW)
                
                elif y < 1000:
                    vx = self.field.value * 600
            elif  self.is_heading_down(1):
                    self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CW)

            self.base_action.move([vx, 0, self.field.value*0.8], is_field=True)

        elif self.state == BehaviorList.ALIVE_ANOTHER_SILO_CW:
            self.base_action.fan.on()

            scoreing_silos = ScoreingSilos(self.silo_camera)
            sorted_silos = scoreing_silos.sort_by_score()
            vx = 0
            if sorted_silos:
                __, y, _ = sorted_silos[0].pos
                if (sorted_silos[0].get_my_team_ball_cnt() + sorted_silos[0].get_opponent_team_cnt()) < 3:
                    self.change_state(BehaviorList.ALIVE_FIND_SILO)
                    pass
                elif self.is_heading_up(1):
                    self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CCW)
                elif y < 1000:
                    vx = self.field.value * 600
            elif self.is_heading_up(1):
                    self.change_state(BehaviorList.ALIVE_ANOTHER_SILO_CCW)

            self.base_action.move([vx, 0, -self.field.value*0.8], is_field=True)          


        
        elif self.state == BehaviorList.ALIVE_CHECK_SILO:
            self.base_action.fan.on()
            rad = self.field.value * pi / 2 - self.posture
            
            if self.silo_camera:
                scoreing_silos = ScoreingSilos(self.silo_camera)
                sorted_silos = scoreing_silos.sort_by_score()
                if (sorted_silos[0].get_my_team_ball_cnt() + sorted_silos[0].get_opponent_team_cnt())>=3:
                    self.change_state(BehaviorList.ALIVE_FIND_SILO)

                x, y, _ = sorted_silos[0].pos
                # print(x, y, _)
                if abs(x) < 100 and self.is_heading_in(0.1*pi):
                    y = y-250
                    self.base_action.fan.hold()
                else:
                    y = 0
                self.follow_object([x, y, rad], (1.2, 1, 2))
            
            else:
                self.change_state(BehaviorList.ALIVE_FIND_SILO)

            if self.wall_sensor_state['Front right'] or self.wall_sensor_state['Front left']:
                # self.change_state(BehaviorList.ALIVE_ALIGN_SILOZONE)
                self.change_state(BehaviorList.ALIVE_PUTIN)

                
        elif self.state == BehaviorList.ALIVE_ALIGN_SILOZONE:
            self.base_action.fan.hold()
            self.move_along_wall(Direction.FRONT)
            if self.wall_sensor_state['Front right'] and self.wall_sensor_state['Front left']:
                self.change_state(BehaviorList.ALIVE_PUTIN)

        elif self.state == BehaviorList.ALIVE_PUTIN:
            self.base_action.fan.off()
            self.base_action.move([0, 0, 0])
            self.base_action.fan.off()
            self.stored_balls = self.stored_balls + 1
            self.change_state(BehaviorList.ALIVE_PUTIN_WAIT)
            if self.stored_balls > 15:
                self.change_state(BehaviorList.FINISH)

        elif self.state == BehaviorList.ALIVE_PUTIN_WAIT:
            self.base_action.fan.off()
            time.sleep(0.5)
            v = [0, -2000, 0]
            for i in range(4):
                self.base_action.move(v)
                time.sleep(0.2)
            self.change_state(BehaviorList.ALIVE_MOVE_TO_STORAGE)
        
        elif self.state == BehaviorList.ALIVE_MOVE_TO_STORAGE:
            if self.is_heading_out(0.8):
                self.change_state(BehaviorList.ALIVE_BALL_SEARCH_CW)
            num, x, y, z, is_obtainable = self.ball_camera
            if num > 0 and self.is_heading_out(1.2):
                self.change_state(BehaviorList.ALIVE_BALL_OBTAINIG_CW)
            else :
                self.base_action.move([self.field.value * 1500, 0, 5*(-self.field.value*pi/2 - self.posture)], is_field=True)   
        
        elif self.state == BehaviorList.FINISH:
            self.shutdown()
                
        return


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
