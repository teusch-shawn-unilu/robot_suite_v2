import enum


class GamepadLogitechF710(enum):
    """Logitech F710 Gamepad Mapping"""

    # Buttons (Digital)
    BTN_A = 0
    BTN_B = 1
    BTN_X = 2
    BTN_Y = 3
    BTN_LB = 4
    BTN_RB = 5
    BTN_BACK = 6
    BTN_START = 7
    BTN_HOME = 8
    BTN_STICK_LEFT = 9
    BTN_STICK_RIGH = 10
    # Axis (Analog)
    AXIS_YAW = 0  # Negative=Left
    AXIS_THROTTLE = 1  # Negative=Up
    AXIS_ROLL = 3  # Negative=Left
    AXIS_PITCH = 4  # Negative=UP
    AXIS_LT = 2
    AXIS_RT = 5
    AXIS_DPAD_HOR = 7  # Negative=Left
    AXIS_DPAD_VER = 6  # Negative=Up
