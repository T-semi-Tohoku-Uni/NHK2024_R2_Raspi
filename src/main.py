from NHK2024_Raspi_Library import MainController, TwoStateButtonHandler, TwoStateButton
from NHK2024_Camera_Library import cam_detect_obj

import json
from typing import Dict, Callable
from enum import Enum
import can

from behavior import Direction, Field, Behavior
from hardware_module import CANList
    

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
    
    def get_received_data(self):
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
        
        self.write(f"Received: {msg.__str__()}")
        self.update_received_can_log(msg)
        #print(f"Received: {msg.__str__()}")


class R2Controller(MainController):
    def __init__(self):
        super().__init__("tsemiR2", 11111, is_udp=False)
        
        # init can message lister
        self.lister = CANMessageLister()
        self.lister.init_write_fnc(self.log_system.write, self.log_system.update_received_can_log, self.log_system.update_send_can_log, self.log_system.update_error_log)
        self.lister.init_write_can_bus_func(self.write_can_bus)
        self.init_can_notifier(lister=self.lister)

        #self.FrontCam0 = cam_detect_obj.FrontCamera(0)
        #self.MainProcess = cam_detect_obj.MainProcess('/home/pi/NHK2024/NHK2024_R2_Raspi/src/NHK2024_Camera_Library/models/20240109best.pt')
        #self.MainProcess.thread_start(self.FrontCam0,self.FrontCam0)

        self.sensor_states = {
                'wall_sensor': {"Right rear": False, "Right front": False, "Front right": False, "Front left": False, "Left front": False, "Left rear": False},
                'posture': [0, 0, 0, 0]
            }
        self.behavior = Behavior(Field.BLUE)
    
    def main(self):
        self.log_system.write(f"Start R2Controller main")
        print(f"Start R2Controller main")
        try:
            while True:
                # raw_ctr_data: Dict = json.loads(self.read_udp()) # read from controller
                # ロボットの中心から見たファンの座標(X,Y)
                FAN_X = cam_detect_obj.OBTAINABE_AREA_CENTER_X
                FAN_Y = cam_detect_obj.OBTAINABE_AREA_CENTER_Y

                # 出力画像は受け取らない
                #_, id, items, x, y, z, is_obtainable = self.MainProcess.q_results.get()
                id, items, x, y, z, is_obtainable = (0, 1, 0, 600, 0, False)                

                '''
                if items == 0 :
                    Vx = 127
                    Vy = 127

                else:
                    Vx = gain[0] * (x - FAN_X) + 127
                    Vy = gain[1] * (y - FAN_Y) + 127
                
                self.write_can_bus(CANList.ROBOT_VEL.value, bytearray([int(Vx), int(Vy), 127]))

                if is_obtainable:
                    self.write_can_bus(CANList.VACUUMFAN.value, bytearray([1]))
                    print("Obtainable!!")
                else:
                    self.write_can_bus(CANList.VACUUMFAN.value, bytearray([0]))     
                 '''
                
                self.parse_from_can_message()
                self.behavior.update_sensor_state(self.sensor_states) 

                #print(self.behavior.sensor_state)
                commands = self.behavior.action()

                #for c in commands:
                #    self.write_can_bus(c[0], c[1])

                self.lister.clear_received_data()


        except KeyboardInterrupt:
            print("KeyboardInterrupt")

    def parse_from_can_message(self) -> None:
        received_datas = self.lister.get_received_data()
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

                self.sensor_states['wall_sensor'] = wall_detection_state
                print(wall_detection_state)
                
    def parse_to_can_message(self, ctr_data: ClientData) -> None:
        # button a
        self.button_a_state.handle_button(
            is_pressed = ctr_data.btn_a,
            action_send_0 = lambda: self.write_can_bus(CANList.VACUUMFAN.value, bytearray([0])),
            action_send_1 = lambda: self.write_can_bus(CANList.VACUUMFAN.value, bytearray([1])),
            # action_send_0 = lambda: print("send 0"), # For debug
            # action_send_1 = lambda: print("send 1"), # For debug
        )
        
        # button b
        self.button_b_state.handle_button(
            is_pressed = ctr_data.btn_b,
            action_send_0 = lambda: self.write_can_bus(CANList.ARM.value, bytearray([0])),
            action_send_1 = lambda: self.write_can_bus(CANList.ARM.value, bytearray([1])),
            # action_send_0 = lambda: print("send 0"), # For debug
            # action_send_1 = lambda: print("send 1"), # For debug
        )
        
        # send v and omega
        self.write_can_bus(CANList.ROBOT_VEL.value, bytearray([ctr_data.v_x, ctr_data.v_y, ctr_data.omega]))
    
if __name__ == "__main__":
    controller = R2Controller()
    controller.main()
