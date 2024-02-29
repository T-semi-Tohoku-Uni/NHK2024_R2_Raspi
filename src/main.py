from NHK2024_Raspi_Library import MainController, TwoStateButtonHandler, TwoStateButton
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
        
    def init_write_func(self, write: Callable[[str], None], write_with_can_id: Callable[[str, int], None]):
        self.write = write
        self.write_with_can_id = write_with_can_id
    
    def on_message_received(self, msg: can.Message):
        can_id: int = msg.arbitration_id
        data: bytearray = msg.data
        
        # write if statement here
        
        # write log file
        self.write(f"Received CAN Message can_id: {can_id}, data: {data}")
        self.write_with_can_id(f"Received CAN Message data: {data}", can_id)
        print(f"Received CAN Message can_id: {can_id}, data: {data}")
    
class R2Controller(MainController):
    def __init__(self, host_name, port):
        super().__init__(host_name=host_name, port=port)
        
        # init can message lister
        lister = CANMessageLister()
        lister.init_write_func(self.log_system.write, self.log_system.write_with_can_id)
        self.init_can_notifier(lister)
        
        # controller button state
        self.button_a_state = TwoStateButtonHandler(state=TwoStateButton.WAIT_1)
        self.button_b_state = TwoStateButtonHandler()
    
    def main(self):
        self.log_system.write(f"Start R2Controller main")
        print(f"Start R2Controller main")
        try:
            while True:
                raw_ctr_data: Dict = json.loads(self.read_udp()) # read from controller
                
                try:
                    ctr_data: ClientData = ClientData(raw_ctr_data) # parse to ClientData
                    self.parse_to_can_message(ctr_data=ctr_data) # parse to can message
                except KeyError as e:
                    self.log_system.write(f"Invalid key is included in the data: {e}")
                    print(f"Invalid key is included in the data: {e}")
                    continue
               
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