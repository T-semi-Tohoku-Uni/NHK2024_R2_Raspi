import can
from enum import Enum


class CANList(Enum):
    EMERGENCY = 0x000
    BATTERY_ERROR = 0x001

    VACUUM_FAN = 0x100
    ARM = 0x101
    ROBOT_VEL = 0x106

    ROBOT_VEL_FB = 0x204
    WALL_DETECTION = 0x205
    LATERAL_SHIFT = 0x206
    ANGLE_DIFF = 0x207
    LINE_DETECT = 0x208


# ハードウェアのモジュールを表す親クラス
class HWBaseModule:
    class BaseState(Enum):
        ERROR = 0
        INIT = 1
        READY = 2
        RUNNING = 3
        STOP = 4

    def __init__(self, can_id: int, can_id_feedback, can_id_error: int = 0x000):
        self.can_bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000, fd=True, data_bitrate=2000000)
        self.can_receive = None
        self.can_id = can_id
        self.can_id_feedback = can_id_feedback
        self.can_id_error = can_id_error

        self.can_message = None
        self.base_state = self.BaseState.INIT
        pass


class Arm(HWBaseModule):
    class ArmState(Enum):
        UP = 0
        DOWN = 1

    def __init__(self):
        super().__init__(CANList.ARM.value, None, CANList.EMERGENCY.value)

        self.state = self.ArmState.DOWN

    def up(self):

        return self.can_id, bytearray([0x00])

    def down(self):
        return self.can_id, bytearray([0x01])

    def get_state(self):
        return self.base_state, self.state


class VacuumFan(HWBaseModule):
    class VacuumFanState(Enum):
        ON = 0
        OFF = 1

    def __init__(self):
        super().__init__(CANList.VACUUM_FAN.value, None, CANList.EMERGENCY.value)

        self.state = self.VacuumFanState.OFF

    def on(self):
        return self.can_id, bytearray([0x01])

    def off(self):
        return self.can_id, bytearray([0x00])

    def get_state(self):
        return self.base_state, self.state