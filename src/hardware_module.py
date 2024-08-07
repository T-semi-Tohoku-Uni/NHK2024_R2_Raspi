import can
from enum import Enum


class Field(Enum):
    BLUE = 1
    RED = -1


class CANList(Enum):
    EMERGENCY = 0x000
    BATTERY_ERROR = 0x001

    START = 0x80

    VACUUM_FAN = 0x100
    ARM = 0x101
    CAM_ARM = 0x102
    ROBOT_VEL = 0x106

    SLOPE_DETECTION = 0x203
    ROBOT_VEL_FB = 0x205
    WALL_DETECTION = 0x204
    LATERAL_SHIFT = 0x206
    ANGLE_DIFF = 0x207
    LINE_DETECT = 0x208

# センサーのリスト
class Sensors(Enum):
    WALL_SENSOR = 'wall_sensor'
    IS_ON_SLOPE = 'is_on_slope'
    BALL_CAMERA = 'ball_camera'
    LINE_CAMERA = 'line_camera'
    SILO_CAMERA = 'silo_camera'
    ROBOT_VEL = 'robot_vel'
    POSTURE = 'posture'
    UI_BUTTONS = 'buttons'
        
"""
'wall_sensor': {"Right rear": False, "Right front": False, "Front right": False, "Front left": False, "Left front": False, "Left rear": False},
'is_on_slope': False,
'ball_camera': (0, 0, 0, 600, False),
'line_camera': (False, False, False, 0),
'robot_vel': [0, 0, 0],
'posture': 0
"""

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
        INIT = 2

    def __init__(self):
        super().__init__(CANList.ARM.value, None, CANList.EMERGENCY.value)

        self.state = None

    def init(self):
        self.state = self.ArmState.INIT
        self.write_can_bus(self.can_id, bytearray([self.ArmState.INIT.value]))

    def up(self):
        self.state = self.ArmState.UP
        self.write_can_bus(self.can_id, bytearray([self.ArmState.UP.value]))
        return self.can_id, bytearray([self.ArmState.UP.value])

    def down(self):
        self.state = self.ArmState.DOWN
        self.write_can_bus(self.can_id, bytearray([self.ArmState.DOWN.value]))
        return self.can_id, bytearray([self.ArmState.DOWN.value])

    def get_state(self):
        return self.base_state, self.state
    

class CamArm(HWBaseModule):
    class CamArmState(Enum):
        UP = 1
        DOWN = 0

    def __init__(self):
        super().__init__(CANList.CAM_ARM.value, None, CANList.EMERGENCY.value)

        self.state = None

    def up(self):
        self.state = self.CamArmState.UP
        self.write_can_bus(self.can_id, bytearray([self.CamArmState.UP.value]))
        return self.can_id, bytearray([self.CamArmState.UP.value])

    def down(self):
        self.state = self.CamArmState.DOWN
        self.write_can_bus(self.can_id, bytearray([self.CamArmState.DOWN.value]))
        return self.can_id, bytearray([self.CamArmState.DOWN.value])

    def get_state(self):
        return self.base_state, self.state


class VacuumFan(HWBaseModule):
    class VacuumFanState(Enum):
        HOLD = 2
        ON = 1
        OFF = 0

    def __init__(self):
        super().__init__(CANList.VACUUM_FAN.value, None, CANList.EMERGENCY.value)

        self.state = None

    def hold(self):
        self.state = self.VacuumFanState.HOLD
        self.write_can_bus(self.can_id, bytearray([self.VacuumFanState.HOLD.value]))
        return self.can_id, bytearray([self.VacuumFanState.HOLD.value])

    def on(self):
        self.state = self.VacuumFanState.ON
        self.write_can_bus(self.can_id, bytearray([self.VacuumFanState.ON.value]))
        return self.can_id, bytearray([0x01])

    def off(self):
        self.state = self.VacuumFanState.OFF
        self.write_can_bus(self.can_id, bytearray([self.VacuumFanState.OFF.value]))
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
