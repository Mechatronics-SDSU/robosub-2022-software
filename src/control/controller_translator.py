"""Translates raw control inputs from pygame into ESC instructions
Maestro values: Anything from -100 to 100
-100 is assumed to be down for Z thrusters, +100 is assumed to be up for Z thrusters

Order of array to send to Maestro:
[Port Forward Z Thruster,
Starboard Forward Z Thruster,
Starboard Aft Z Thruster,
Port Aft Z Thruster,
Port Forward Vectored Thruster,
Port Aft Vectored Thruster,
Starboard Forward Vectored Thruster,
Starboard Aft Vectored Thruster]

[PFZT, SFZT, SAZT, PAZT, PFVT, PAVT, SFVT, SAVT]
Diagram:
                                     Bow/Front
      Port Forward Z Thurster PFZT ->  0===0 <- Starboard Forward Z Thurster SFZT
Port Forward Vectored Thruster PFVT-> // | \\ <- Starboard Forward Vectored Thruster SFVT
                                      || | ||
   Port Aaft Vectored Thruster PAVT-> \\ | // <- Starboard Aft Vectored Thruster SAVT
          Port Aft Z Thurster PAZT ->  0===0 <- Starboard Aft Z Thruster SAZT
                                     Stern/Aft

Testing on 4/26-5/6 (IAR):
Forward movement: Return values from function are 1/4th expected on vectored thrusters only
    5/7 Fixed and tested -IAR
Aft movement: Return values from function are 1/4th expected on vectored thrusters only
    5/7 Fixed and tested -IAR
Port Strafe: Z thrusters engaged instead of vectored thrusters. Opposite thrusters to strafe direction only 1/5th of
    expected value.
Starboard Strafe: Z thrusters engaged instead of vectored thrusters. Opposite thrusters to strafe direction only 1/5th
    of expected value.
In Place Port Turn: V Thrusters not engaged, all values are 0. Expected RJ_X.
In Place Starboard Turn: V Thrusters not engages, all values are 0. Expected -RJ_X.
Port Turn: V Thrusters absolute values are correct, but expected values are inverted.
Starboard Turn: V Thrusters absolute values are correct, SFVT is correct, PFVT and SAVT are inverted.
Inv Port Turn: V Thrusters incorrectly mapped, PAVT and SFVT are driving LJ_Y in spec, code is driving PFVT and SAVT,
    and are in the wrong direction. PFV is wrong direction and mapped incorrectly to SFV.
Inv Starboard Turn: V Thrusters incorrectly mapped, PFVT and SAVT are driving LJ_Y in spec, code is driving PAVT and
    SFVT, and are in the wrong direction. SFVT is not even coded in.
45 Port Strafe: All Vectored thrusters engaged at half of expected value. Z Thrusters engaged when not expected.
45 Starboard Strafe: All Vectored thrusters engaged at half of expected value. Z Thrusters engaged when not expected.
45 Inverted Port Strafe: All Vectored thrusters engaged at half of expected value. Z Thrusters engaged when not
    expected.
45 Inverted Starboard Strafe: All Vectored thrusters engaged at half of expected value. Z Thrusters engaged when not
    expected.

"""

import math
import numpy as np


class ControllerTranslator:
    """Gets controller state as a numpy array and translates it into controls sent to the maestro.
    """

    def __init__(self,
                 offset=0,
                 invert_controls=False,
                 joystick_drift_compensation=0.05,
                 base_net_turn=0,
                 z_drift_compensation=0,
                 base_net_strafe=0,
                 debug=False):
        self.invert = invert_controls
        self.joystick_drift_compensation = joystick_drift_compensation
        self.base_net_turn = base_net_turn
        self.z_drift_compensation = z_drift_compensation
        self.base_net_strafe = base_net_strafe
        self.offset = offset  # amount to offset ESCs by when performing translation
        self.debug = debug
        # Ex. ESC needs value of 50 to begin moving thrusters. Offset of 49 means 0 is mapped to 49

    def translate_to_maestro_controller(self, inputs) -> list:
        """Accepts a numpy array from pygame controller, translates into maestro instructions.
        :return: list of instructions for the maestro
        """

        # Z
        L2 = inputs[0][4]
        R2 = inputs[0][5]
        # XY
        LJ_X = inputs[0][0]
        LJ_Y = inputs[0][1]
        RJ_X = inputs[0][2]
        RJ_Y = inputs[0][3]  # Left in for future funtionality -IAR 5/7/22

        # Calculate Cartesian

        # Calculate Z
        z_abs = 0
        z_dir = 0
        # L2 and R2 are mutually exclusive, one > -1 means other is -1.
        # Shift from range -1, 1 to 0, 2 such that -1 is mapped to 0 for rest state
        if L2 > -1:
            L2 += 1
            z_abs = L2 / 2  # Divide by 2 to get range from 0, 1
            z_dir = 1  # L2 mapped to up
        else:
            R2 += 1
            z_abs = R2 / 2  # Divide by 2 to get range from 0, 1
            z_dir = -1  # R2 mapped to down
        # z_abs now a percentile of how far to move, z_dir is positive if up and negative if down

        # Cartesian quadrant the joysticks are in
        quadrant_LJ = 0
        if ((LJ_X >= 0) and (LJ_Y < 0)) and \
                ((LJ_X > self.joystick_drift_compensation) or (math.fabs(LJ_Y) > self.joystick_drift_compensation)):
            quadrant_LJ = 1
        elif ((LJ_X < 0) and (LJ_Y < 0)) and \
                ((math.fabs(LJ_X) > self.joystick_drift_compensation) or (
                        math.fabs(LJ_Y) > self.joystick_drift_compensation)):
            quadrant_LJ = 2
        elif ((LJ_X < 0) and (LJ_Y >= 0)) and \
                ((math.fabs(LJ_X) > self.joystick_drift_compensation) or (LJ_Y > self.joystick_drift_compensation)):
            quadrant_LJ = 3
        elif ((LJ_X >= 0) and (LJ_Y >= 0)) and \
                ((LJ_X > self.joystick_drift_compensation) or (LJ_Y > self.joystick_drift_compensation)):
            quadrant_LJ = 4

        if self.debug:
            print(f'DEBUG: L2: {L2} | R2: {R2} | RJ_X: {RJ_X} | LJ_X: {LJ_X} | LJ_Y: {LJ_Y} | Quadrant LJ: '
                  f'{quadrant_LJ}')

        # Translate

        # SFVT/PFVT/SAVT/PAVT are a function of LJY and RJX
        SFVT = 0
        PFVT = 0
        PAVT = 0
        SAVT = 0
        delta = 100 - self.offset
        if ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (
                math.fabs(RJ_X) <= self.joystick_drift_compensation):  # Forward
            if delta < 100:  # Map proportionally starting at offset instead of 0
                SFVT = math.floor((self.offset + math.floor(math.fabs(LJ_Y) * delta)))

            else:
                SFVT = math.floor((math.fabs(LJ_Y) * 100))
            PFVT = SFVT
            PAVT = SFVT
            SAVT = SFVT  # Going forward, both motors should be same values. Dividing by 4 since there are 4 motors to control this direction

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (
                math.fabs(RJ_X) <= self.joystick_drift_compensation):  # Backward
            if delta < 100:
                SFVT = math.floor(self.offset + math.ceil(-1 * LJ_Y * delta))
            else:
                SFVT = math.ceil((-1 * LJ_Y * 100))
            PFVT = SFVT
            PAVT = SFVT
            SAVT = SFVT  # going backward

        elif ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (
                RJ_X > self.joystick_drift_compensation):  # Turn to starboard
            PFVT = SAVT = math.floor(LJ_Y * 100)
            SFVT = math.ceil(RJ_X * -100)
            PAVT = 0

        elif ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (
                RJ_X < (-1 * self.joystick_drift_compensation)):  # Turn to port
            PFVT = math.ceil(RJ_X * -100)
            PAVT = SFVT = math.floor(LJ_Y * 100)
            SAVT = 0

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (
                RJ_X > self.joystick_drift_compensation):  # Inverted turn to port
            SAVT = 0
            PAVT = SFVT = math.floor(LJ_Y * 100)
            PFVT = math.floor(RJ_X)

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (
                RJ_X < (-1 * self.joystick_drift_compensation)):  # Inverted turn to starboard
            PAVT = 0
            PFVT = SAVT = math.floor(LJ_Y * 100)
            SFVT = math.ceil(RJ_X * -100)

        elif (math.fabs(LJ_X) > self.joystick_drift_compensation) and (
                LJ_Y < self.joystick_drift_compensation):  # Strafe Starboard
            SFVT = PAVT = self.offset + math.ceil(LJ_X * -100)
            PFVT = SAVT = self.offset + math.floor(LJ_X * 100)

        elif (math.fabs(LJ_X) < self.joystick_drift_compensation) and (
                LJ_Y < self.joystick_drift_compensation):  # Strafe Port
            SFVT = PAVT = math.floor(LJ_X * 100)
            PFVT = SAVT = math.ceil(LJ_X * -100)

        elif ((LJ_Y > self.joystick_drift_compensation) and (LJ_X < (-1 * self.joystick_drift_compensation)) and (
                RJ_X < self.joystick_drift_compensation)):  # 45 Port Strafe
            PFVT = 0
            SAVT = 0
            PAVT = SFVT = ((math.floor((LJ_Y * 100) + math.ceil(LJ_X * -100))) / 2)

        elif ((LJ_Y > self.joystick_drift_compensation) and (LJ_X < self.joystick_drift_compensation) and (
                RJ_X < self.joystick_drift_compensation)):  # 45 Starboard Strafe
            PAVT = SFVT = 0
            PFVT = SAVT = math.floor(((LJ_Y * 100) + (LJ_X * 100)) / 2)

        elif ((LJ_Y < (-1 * self.joystick_drift_compensation)) and (
                LJ_X < (-1 * self.joystick_drift_compensation)) and (
                      RJ_X < self.joystick_drift_compensation)):  # 45 Inverted Port Strafe
            PFVT = SAVT = 0
            PAVT = SFVT = math.floor(((LJ_Y * 100) + (LJ_X * 100)) / 2)

        elif ((LJ_Y < (-1 * self.joystick_drift_compensation)) and (LJ_X > self.joystick_drift_compensation) and (
                RJ_X < self.joystick_drift_compensation)):  # 45 Inverted Starboard Strafe
            PFVT = SAVT = ((math.ceil(LJ_Y * -100) + math.floor(LJ_X * 100)) / 2)
            PAVT = SFVT = 0

        elif (math.fabs(RJ_X) > self.joystick_drift_compensation) and (RJ_X > 0):  # Turn in-place to starboard
            if delta < 100:
                SFVT = SAVT = self.offset + math.ceil(RJ_X * -1 * delta)  # Reverse on Starboard Y Thruster
                PFVT = PAVT = self.offset + math.floor(RJ_X * delta)  # Forward on Port Y Thruster

            else:
                SYT = math.ceil(RJ_X * -100)  # FIXME
                PYT = math.floor(RJ_X * 100)

        elif (math.fabs(RJ_X) > self.joystick_drift_compensation) and (RJ_X < 0):  # Turn in-place to port
            if delta < 100:
                SFVT = self.offset + math.floor(RJ_X * delta)  # Forward on Starboard Y Thruster
                PAFT = SFVT
                PFVT = self.offset + math.ceil(RJ_X * -1 * delta)  # Reverse on Port Y Thruster
                PAVT = PFVT
            else:
                SYT = math.floor(RJ_X * -100)
                PYT = math.ceil(RJ_X * 100)
        else:  # No movement
            SFVT = 0
            SAVT = 0
            PFVT = 0
            PAVT = 0

        # PFZT, SFZT, SAZT, PAZT are a function of LJ_X and L2/R2
        PFZT = 0
        SFZT = 0
        SAZT = 0
        PAZT = 0

        if (z_abs > self.z_drift_compensation) and (z_dir == 1):  # Ascend
            PFZT = SFZT = SAZT = PAZT = math.floor(100 * z_abs)
        elif (z_abs > self.z_drift_compensation) and (z_dir == -1):  # Descend
            PFZT = SFZT = SAZT = PAZT = math.ceil(-100 * z_abs)
        elif (LJ_X > self.joystick_drift_compensation) and (z_abs <= self.z_drift_compensation):  # Strafe to Starboard
            if delta < 100:
                SFZT = SAZT = (-1 * self.offset) + math.ceil(LJ_X * -1 * delta)
                PAZT = PFZT = (-1 * self.offset) + self.base_net_strafe
            else:
                SFZT = SAZT = math.ceil(LJ_X * -100)
                PAZT = PFZT = math.ceil(self.base_net_strafe)
        elif ((-1 * LJ_X) > self.joystick_drift_compensation) and (
                z_abs <= self.z_drift_compensation):  # Strafe to Port
            if delta < 100:
                PAZT = PFZT = (-1 * self.offset) + math.ceil(LJ_X * -1 * delta)
                SFZT = SAZT = (-1 * self.offset) + self.base_net_strafe
            else:
                PAZT = PFZT = math.ceil(LJ_X * 100)
                SFZT = SAZT = math.ceil(self.base_net_strafe)

        return [PFZT, SFZT, SAZT, PAZT, PFVT, PAVT, SFVT, SAVT]


def _driver_test_code() -> None:
    """Test code using controller inputs directly. Don't run in other modules!"""
    import pygame as pg
    pg.init()
    pg.joystick.init()
    js = pg.joystick.Joystick(0)
    js.init()
    print(str(js.get_numaxes()) + ' ' + str(js.get_numbuttons()) + ' ' + str(js.get_numhats()))
    ct = ControllerTranslator(joystick_drift_compensation=0.1, base_net_turn=10, base_net_strafe=-20, debug=True)
    while True:
        if js.get_init():
            control_in = np.zeros(shape=(1, js.get_numaxes()
                                         + js.get_numbuttons()
                                         + js.get_numhats()))
            for i in range(js.get_numaxes()):
                control_in.put(i, js.get_axis(i))
            for i in range(js.get_numaxes(), js.get_numbuttons()):  # Buttons
                control_in.put(i, js.get_button(i - js.get_numaxes()))

            control_in.put((js.get_numaxes() + js.get_numbuttons()), js.get_hat(0))  # Hat
            result = ct.translate_to_maestro_controller(control_in)
            print(f"PFZ: {result[0]} SFZ: {result[1]} SAZ: {result[2]} PAZ: {result[3]} PFV: {result[4]} "
                  f"PAV: {result[5]} SFV: {result[6]} SAV: {result[7]}")
        pg.event.pump()


if __name__ == '__main__':
    _driver_test_code()
else:
    print('Initialized Controller Translator module')
