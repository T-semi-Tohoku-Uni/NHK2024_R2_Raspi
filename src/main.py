from NHK2024_Raspi_Library import MainController, TwoStateButtonHandler, TwoStateButton
from NHK2024_Camera_Library import cam_detect_obj

import json
from typing import Dict, Callable
from enum import Enum
import can

class CANList(Enum):
    EMERGENCY=0x000
    BATTERY_ERROR=0x001
    
    VACUUMFAN=0x100
    ARM=0x101
    ROBOT_VEL=0x106
    
    WALL_DETECTION=0x205
    LATERAL_SHIFT=0x206
    ANGLE_DIFF=0x207
    LINE_DETECT=0x208
    

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
    
    def init_write_fnc(self, write: Callable[[str], None], update_received_can_log: Callable[[can.Message], None], update_send_can_log: Callable[[can.Message], None], update_error_log: Callable[[str], None]):
        self.write = write
        self.update_received_can_log = update_received_can_log
        self.update_send_can_log = update_send_can_log
        self.update_error_log = update_error_log
    
    def init_write_can_bus_func(self, write_can_bus: Callable[[int, bytearray], None]):
        self.write_can_bus = write_can_bus
    
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
        
        self.write(f"Received: {msg.__str__()}")
        self.update_received_can_log(msg)
        print(f"Received: {msg.__str__()}")
    
class R2Controller(MainController):
    def __init__(self, host_name, port):
        super().__init__(host_name=host_name, port=port)
        
        # init can message lister
        lister = CANMessageLister()
        lister.init_write_fnc(self.log_system.write, self.log_system.update_received_can_log, self.log_system.update_send_can_log, self.log_system.update_error_log)
        lister.init_write_can_bus_func(self.write_can_bus)
        self.init_can_notifier(lister=lister)

        self.FrontCam0 = cam_detect_obj.FrontCamera('src/NHK2024_Camera_Library/models/20240109best.pt', 0)
        
    
    def main(self):
        self.log_system.write(f"Start R2Controller main")
        print(f"Start R2Controller main")
        try:
            while True:
                # raw_ctr_data: Dict = json.loads(self.read_udp()) # read from controller
                if self.FrontCam0.cap.isOpened():
                    W = 0
                    H = 550
                    gain = [0.1, -0.1, 1]

                    items, x, y, z, is_obtainable = self.FrontCam0.queue.get()

                    print(x, y, z)


                    if items == 0 :
                        Vx = 127
                        Vy = 127

                    else:
                        Vx = gain[0] * (x - W / 2) + 127
                        Vy = gain[1] * (y - H / 2) + 127
                    
                    self.write_can_bus(CANList.ROBOT_VEL.value, bytearray([int(Vx), int(Vy), 127]))

                    if is_obtainable:
                        self.write_can_bus(CANList.VACUUMFAN.value, bytearray([1]))
                        print("Obtainable!!")
                    else:
                        self.write_can_bus(CANList.VACUUMFAN.value, bytearray([0]))
                                       
        except KeyboardInterrupt:
            print("KeyboardInterrupt")
        

    
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
    host_name = "R2.local"
    port = 12345
    controller = R2Controller(host_name=host_name, port=port)
    controller.main()