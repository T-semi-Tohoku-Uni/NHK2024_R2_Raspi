from NHK2024_Raspi_Library import MainController, TwoStateButtonHandler, TwoStateButton
from NHK2024_Camera_Library import MainProcess, OUTPUT_ID, LINE_SLOPE_THRESHOLD, OBTAINABE_AREA_CENTER_X, OBTAINABE_AREA_CENTER_Y
import json
from typing import Dict, Callable
from enum import Enum
import can
import cv2

import time

from behavior import Direction, Field, Behavior, BehaviorList
from hardware_module import CANList,Sensors
    

field = Field.BLUE

class ClientData:
    def __init__(self, data: Dict):
        try:
            self.v_x = data["v_x"]
            self.v_y = data["v_y"]
            self.omega = data["omega"]
            self.btn_a = data["btn_a"]
            self.btn_b = data["btn_b"]
            self.btn_x = data["btn_x"]
            self.btn_y = data["btn_y"]
        except KeyError as e:
            raise KeyError("Invalid key is included in the data: {e}")


class CANMessageLister(can.Listener):
    def __init__(self):
        super().__init__()
        self.write = None
        self.write_with_can_id = None
        self.received_datas = [] 
    def init_write_fnc(self, write: Callable[[str], None], update_received_can_log: Callable[[can.Message], None], update_send_can_log: Callable[[can.Message], None], update_error_log: Callable[[str], None]):
        self.write = write
        self.update_received_can_log = update_received_can_log
        self.update_send_can_log = update_send_can_log
        self.update_error_log = update_error_log
    
    def init_write_can_bus_func(self, write_can_bus: Callable[[int, bytearray], None]):
        self.write_can_bus = write_can_bus

    def store_received_data(self, msg: can.Message):
        self.received_datas.append(msg)
    
    def get_received_datas(self):
        return self.received_datas

    def clear_received_data(self):
        self.received_datas.clear()
    
    def on_message_received(self, msg):
        can_id: int = int(msg.arbitration_id)
        data: str = msg.data.hex()
        is_error: bool = msg.is_error_frame
        
        # Write if statement          

        if is_error is True:
            self.update_error_log(msg.__str__())
            print(f"Get Error Frame: {msg.__str__()}")
        
        # write log file
        if self.write is None or self.update_send_can_log is None or self.update_received_can_log is None:
            print("write function is not initialized")
            return

        self.store_received_data(msg)

        # self.write(f"Received: {msg.__str__()}")
        self.update_received_can_log(msg)
        #print(f"Received: {msg.__str__()}")


class R2Controller(MainController):
    def __init__(self):
        super().__init__("tsemiR2", 11111, is_udp=False)
        self.behavior = Behavior(field, (OBTAINABE_AREA_CENTER_X, OBTAINABE_AREA_CENTER_Y), 
                                #  start_state=BehaviorList.ALIVE_AREA3_FIRST_ATTEMPT,
                                #  finish_state=BehaviorList.ALIVE_PUTIN_WAIT
                                 )
        
        # init can message lister
        self.lister = CANMessageLister()
        self.lister.init_write_fnc(self.log_system.write, self.log_system.update_received_can_log, self.log_system.update_send_can_log, self.log_system.update_error_log)
        self.lister.init_write_can_bus_func(self.write_can_bus)
        self.init_can_notifier(lister=self.lister)

        self.behavior.init_log_system(self.log_system)
        self.behavior.init_write_can_bus(self.write_can_bus)

        self.is_running = False

        self.sensor_states = {
                Sensors.WALL_SENSOR: {"Right rear": False, "Right front": False, "Front right": False, "Front left": False, "Left front": False, "Left rear": False},
                Sensors.IS_ON_SLOPE: False,
                Sensors.BALL_CAMERA: (0, 0, 0, 600, False),
                Sensors.LINE_CAMERA: (False, False, False, 0),
                Sensors.SILO_CAMERA: (0, 0, 0, 0, 0),
                Sensors.ROBOT_VEL: [0, 0, 0],
                Sensors.POSTURE: 0,
                Sensors.UI_BUTTONS: 0
            }
        
        # メインプロセスを実行するクラス
        print("create main instance")
        self.mainprocess = MainProcess(
            field=field,
            save_image_dir=self.log_system.get_log_dir(),
            show=True
        )
        
        # マルチスレッドの実行
        print("start thread")
        self.mainprocess.thread_start()
        print("complete start thread")
    
    def main(self):
        self.log_system.write(f"Start R2Controller main")
        print(f"Start R2Controller main")
        
        try:
            while True:
                self.sensor_states[Sensors.BALL_CAMERA] = self.mainprocess.update_ball_camera_out()
                self.sensor_states[Sensors.LINE_CAMERA] = self.mainprocess.update_line_camera_out()
                self.sensor_states[Sensors.SILO_CAMERA] = self.mainprocess.update_silo_camera_out()
                # print(self.sensor_states[Sensors.BALL_CAMERA])
                # print(self.sensor_states[Sensors.SILO_CAMERA])
                # silos = self.sensor_states[Sensors.SILO_CAMERA]
                # print('loop')
                # for i, silo in enumerate(silos):
                #     print(f'silo:{i}: {silo.pos}')
                frame, id = self.mainprocess.q_out.get() 

                # # 画面表示用
                # cv2.imshow(f'{id}', frame)
                # key = cv2.waitKey(1)
                # if key == ord("q"):
                #     break
                # # ここまで

                # print(f'up:{self.behavior.is_heading_up(0.1)}')
                # print(f'down:{self.behavior.is_heading_down(0.1)}')
                # print(f'in:{self.behavior.is_heading_in(0.1)}')
                # print(f'out:{self.behavior.is_heading_out(0.1)}')
                # print(f'posture:{self.behavior.posture}')

                
                # print(f"hello {self.mainprocess.update_ball_camera_out()}")
                self.parse_from_can_message()
                self.lister.clear_received_data()
                self.behavior.update_sensor_state(self.sensor_states)
                self.behavior.action()

                time.sleep(0.01)
                continue

        except KeyboardInterrupt:
            print("KeyboardInterrupt")
            self.mainprocess.finish()
            self.behavior.shutdown()

    def parse_from_can_message(self) -> None:
        received_datas = self.lister.get_received_datas()
        for data in received_datas:
            can_id: int = int(data.arbitration_id)
            if can_id == CANList.WALL_DETECTION.value:
                wall_detection_state = {
                    "Front right": not(bool(data.data[0] & 0x20)), 
                    "Front left": not(bool(data.data[0] & 0x10)), 
                    "Right front": not(bool(data.data[0] & 0x40)), 
                    "Right rear": not(bool(data.data[0] & 0x80)),  
                    "Left front": not(bool(data.data[0] & 0x08)), 
                    "Left rear": not(bool(data.data[0] & 0x04))
                    }

                self.sensor_states[Sensors.WALL_SENSOR] = wall_detection_state

            elif can_id == CANList.SLOPE_DETECTION.value:
                self.sensor_states[Sensors.IS_ON_SLOPE] = bool(data.data[0])

            elif can_id == CANList.ROBOT_VEL_FB.value:
                self.sensor_states[Sensors.ROBOT_VEL] = [(data.data[0] - 127) * 16, (data.data[1] - 127) * 16, (data.data[2] - 127) * 0.02]
                self.sensor_states[Sensors.POSTURE] = (data.data[3] - 127) / 40
                # print('posture:', self.sensor_states['posture'])
            elif can_id == CANList.START.value:
                self.sensor_states[Sensors.UI_BUTTONS] = data.data[0]
    
if __name__ == "__main__":
    controller = R2Controller()
    controller.main()
