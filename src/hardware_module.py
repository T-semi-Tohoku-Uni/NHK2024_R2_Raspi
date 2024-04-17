import can
from enum import Enum


class CANList(Enum):
    EMERGENCY = 0x000
    BATTERY_ERROR = 0x001

    VACUUM_FAN = 0x100
    ARM = 0x101
    ROBOT_VEL = 0x106

    SLOPE_DETECTION = 0x203
    ROBOT_VEL_FB = 0x205
    WALL_DETECTION = 0x204
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
        self.can_receive = None
        self.can_id = can_id
        self.can_id_feedback = can_id_feedback
        self.can_id_error = can_id_error

        self.write_can_bus = None

        self.can_message = None
        self.base_state = self.BaseState.INIT
        pass

    def init_write_can_bus_func(self, write_can_bus):
        self.write_can_bus = write_can_bus


class Arm(HWBaseModule):
    class ArmState(Enum):
        UP = 0
        DOWN = 1

    def __init__(self):
        super().__init__(CANList.ARM.value, None, CANList.EMERGENCY.value)

        self.state = None

    def up(self):
        self.state = self.ArmState.UP
        self.write_can_bus(self.can_id, bytearray([0x00]))
        return self.can_id, bytearray([0x00])

    def down(self):
        self.state = self.ArmState.DOWN
        self.write_can_bus(self.can_id, bytearray([0x01]))
        return self.can_id, bytearray([0x01])

    def get_state(self):
        return self.base_state, self.state


class VacuumFan(HWBaseModule):
    class VacuumFanState(Enum):
        ON = 0
        OFF = 1

    def __init__(self):
        super().__init__(CANList.VACUUM_FAN.value, None, CANList.EMERGENCY.value)

        self.state = None

    def on(self):
        self.state = self.VacuumFanState.ON
        self.write_can_bus(self.can_id, bytearray([0x00]))
        return self.can_id, bytearray([0x01])

    def off(self):
        self.state = self.VacuumFanState.OFF
        self.write_can_bus(self.can_id, bytearray([0x01]))
        return self.can_id, bytearray([0x00])

    def get_state(self):
        return self.base_state, self.state


class Position(Enum):
    RIGHT_REAR = 0
    RIGHT_FRONT = 1
    FRONT_RIGHT = 2
    FRONT_LEFT = 3
    LEFT_FRONT = 4
    LEFT_REAR = 5


class WallSensors:            # このクラスはWallSensorクラスのリストを持つ
    class WallSensor(HWBaseModule):
        class WallSensorState(Enum):
            ON = 0
            OFF = 1

        def __init__(self, can_id: int, can_id_feedback: int, position: Position):
            super().__init__(can_id, can_id_feedback, CANList.EMERGENCY.value)
            self.position = position
            self.state = self.WallSensorState.OFF

        def set_state(self, state: WallSensorState):
            self.state = state

        def get_state(self):
            return self.base_state, self.state

    def __init__(self):
        super().__init__(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, CANList.EMERGENCY.value)

        self.list = [
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.RIGHT_REAR),
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.RIGHT_FRONT),
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.FRONT_RIGHT),
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.FRONT_LEFT),
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.LEFT_FRONT),
            self.WallSensor(CANList.WALL_DETECTION.value, CANList.WALL_DETECTION.value, Position.LEFT_REAR)
        ]

        self.state_list = [sensor.get_state() for sensor in self.list]

    def get_state(self):
        return self.state_list

    def set_state(self, state_list):
        for i, state in enumerate(state_list):
            self.list[i].set_state(state)
            self.state_list[i] = self.list[i].get_state()
