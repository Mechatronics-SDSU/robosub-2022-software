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

SOFTWARE:
[PFZT, SFZT, SAZT, PAZT, PFVT, PAVT, SFVT, SAVT]
HARDWARE:
[PFZT, PFVT, PAZT, PAVT, SAZT, SAVT, SFZT, SFVT]
Diagram:
                                     Bow/Front
      Port Forward Z Thurster PFZT ->  0===0 <- Starboard Forward Z Thurster SFZT
Port Forward Vectored Thruster PFVT-> // | \\ <- Starboard Forward Vectored Thruster SFVT
                                      || | ||
   Port Aft Vectored Thruster PAVT-> \\ | // <- Starboard Aft Vectored Thruster SAVT
          Port Aft Z Thurster PAZT ->  0===0 <- Starboard Aft Z Thruster SAZT
                                     Stern/Aft

Testing on 4/26-5/7 (IAR):
Confirmed all movement configs 5/7 3:25AM -IAR
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
        L2 = inputs[0][2] / 1.5
        R2 = inputs[0][5] / 1.5
        # XY
        LJ_X = inputs[0][0] / 1.5
        LJ_Y = inputs[0][1] / 1.5
        RJ_X = inputs[0][3] / 1.5

        '''
        WINDOWS ONLY TEMP FIXING FOR LINUX
        # Z
        L2 = inputs[0][4]
        R2 = inputs[0][5]
        # XY
        LJ_X = inputs[0][0]
        LJ_Y = inputs[0][1]
        RJ_X = inputs[0][2]
        RJ_Y = inputs[0][3]  # Left in for future funtionality -IAR 5/7/22
        '''

        # Calculate Cartesian

        # Calculate Z
        z_dir = 0
        # L2 and R2 are mutually exclusive but will be summed to average
        # Shift from range -1, 1 to 0, 2 such that -1 is mapped to 0 for rest state
        L2 += 1
        R2 += 1
        z_abs_pos = L2 / 2  # Divide by 2 to get range from 0, 1
        z_abs_neg = R2 / 2  # Divide by 2 to get range from 0, 1
        z_abs = 0
        if math.fabs(z_abs_pos) > math.fabs(z_abs_neg):
            z_dir = 1
            z_abs = z_abs_pos - z_abs_neg
        else:
            z_dir = -1
            z_abs = z_abs_neg - z_abs_pos
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

        #if self.debug:
            #print(f'DEBUG: L2: {L2} | R2: {R2} | RJ_X: {RJ_X} | LJ_X: {LJ_X} | LJ_Y: {LJ_Y} | Quadrant LJ: '
            #      f'{quadrant_LJ}')

        # Translate

        # SFVT/PFVT/SAVT/PAVT are a function of LJY and RJX
        SFVT = 0
        PFVT = 0
        PAVT = 0
        SAVT = 0
        delta = 100 - self.offset
        if ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (math.fabs(LJ_Y) > self.joystick_drift_compensation) and (
                math.fabs(LJ_X) <= self.joystick_drift_compensation * 4) and (
                math.fabs(RJ_X) <= self.joystick_drift_compensation * 4):  # Forward
            if delta < 100:  # Map proportionally starting at offset instead of 0
                SFVT = math.floor((self.offset + math.floor(math.fabs(LJ_Y) * delta)))

            else:
                SFVT = math.floor((math.fabs(LJ_Y) * 100))
            PFVT = SFVT
            PAVT = -1 * SFVT
            SAVT = -1 * SFVT  # Going forward, both motors should be same values.

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (math.fabs(LJ_Y) > self.joystick_drift_compensation) and (
                math.fabs(LJ_X) <= self.joystick_drift_compensation * 4) and (
                math.fabs(RJ_X) <= self.joystick_drift_compensation * 4):  # Backward
            if delta < 100:
                SFVT = math.floor(self.offset + math.ceil(-1 * LJ_Y * delta))
            else:
                SFVT = math.ceil((-1 * LJ_Y * 100))
            PFVT = SFVT
            PAVT = -1 * SFVT
            SAVT = -1 * SFVT  # going backward

        elif ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (
                RJ_X > self.joystick_drift_compensation):  # Turn to starboard
            PFVT = SAVT = math.floor(LJ_Y * -100)
            SFVT = math.ceil(RJ_X * -100)
            PAVT = 0

        elif ((quadrant_LJ == 1) or (quadrant_LJ == 2)) and (
                RJ_X < (-1 * self.joystick_drift_compensation)):  # Turn to port
            PFVT = math.ceil(RJ_X * 100)
            PAVT = SFVT = math.floor(LJ_Y * -100)
            SAVT = 0

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (
                RJ_X < -1 * self.joystick_drift_compensation):  # Inverted turn to port
            SAVT = 0
            PAVT = SFVT = math.floor(LJ_Y * -100)
            PFVT = math.floor(RJ_X * 100)

        elif ((quadrant_LJ == 3) or (quadrant_LJ == 4)) and (
                RJ_X > self.joystick_drift_compensation):  # Inverted turn to starboard
            PAVT = 0
            PFVT = SAVT = math.floor(LJ_Y * -100)
            SFVT = math.ceil(RJ_X * -100)

        elif (math.fabs(LJ_X) > self.joystick_drift_compensation) and (quadrant_LJ == 1 or quadrant_LJ == 4) and (
                math.fabs(LJ_Y) <= (self.joystick_drift_compensation * 2)):  # Strafe Starboard
            SFVT = PAVT = self.offset + math.ceil(LJ_X * -100)
            PFVT = SAVT = self.offset + math.floor(LJ_X * 100)

        elif (math.fabs(LJ_X) > self.joystick_drift_compensation) and (quadrant_LJ == 2 or quadrant_LJ == 3) and (
                math.fabs(LJ_Y) <= (self.joystick_drift_compensation * 2)):  # Strafe Port
            SFVT = PAVT = self.offset + math.floor(LJ_X * -100)  # Make positive
            PFVT = SAVT = self.offset + math.ceil(LJ_X * 100)

        elif ((math.fabs(LJ_Y) > self.joystick_drift_compensation) and (LJ_X < (-1 * self.joystick_drift_compensation))
              and (math.fabs(RJ_X) < self.joystick_drift_compensation) and (quadrant_LJ == 2)):  # 45 Port Strafe
            PFVT = 0
            SAVT = 0
            PAVT = SFVT = math.fabs((math.floor(((math.fabs(LJ_Y) * 100) + math.fabs(LJ_X) * 100) / 2)))

        elif ((math.fabs(LJ_Y) > self.joystick_drift_compensation) and (LJ_X > self.joystick_drift_compensation) and (
                math.fabs(RJ_X) < self.joystick_drift_compensation) and (quadrant_LJ == 1)):  # 45 Starboard Strafe
            PAVT = SFVT = 0
            PFVT = SAVT = math.fabs((math.floor(((math.fabs(LJ_Y) * 100) + math.fabs(LJ_X) * 100) / 2)))

        elif ((math.fabs(LJ_Y) > self.joystick_drift_compensation) and (LJ_X < (-1 * self.joystick_drift_compensation))
              and (math.fabs(RJ_X) < self.joystick_drift_compensation) and (quadrant_LJ == 3)):
            # 45 Inverted Port Strafe
            PFVT = SAVT = 0
            PAVT = SFVT = -1 * math.fabs((math.floor(((math.fabs(LJ_Y) * 100) + math.fabs(LJ_X) * 100) / 2)))

        elif ((math.fabs(LJ_Y) > self.joystick_drift_compensation) and (LJ_X > self.joystick_drift_compensation) and (
                math.fabs(RJ_X) < self.joystick_drift_compensation) and (quadrant_LJ == 4)):
            # 45 Inverted Starboard Strafe
            PFVT = SAVT = -1 * math.fabs((math.floor(((math.fabs(LJ_Y) * 100) + math.fabs(LJ_X) * 100) / 2)))
            PAVT = SFVT = 0

        elif (math.fabs(RJ_X) > self.joystick_drift_compensation) and (RJ_X > 0):  # Turn in-place to starboard
            SFVT = self.offset + math.floor(RJ_X * -1 * delta)  # Reverse on Starboard V Thrusters
            SAVT = SFVT
            PFVT = self.offset + math.ceil(RJ_X * delta)  # Forward on Port V Thrusters
            PAVT = PFVT

        elif (math.fabs(RJ_X) > self.joystick_drift_compensation) and (RJ_X < 0):  # Turn in-place to port
            SFVT = self.offset + math.floor(RJ_X * -1 * delta)  # Forward on Starboard V Thrusters
            SAVT = SFVT
            PFVT = self.offset + math.ceil(RJ_X * delta)  # Reverse on Port V Thrusters
            PAVT = PFVT

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

        #return [int(PFZT), int(SFZT), int(SAZT), int(PAZT),
        # int(PFVT), int(PAVT), int(SFVT), int(SAVT)]
        return [int(PFZT), int(PFVT), int(PAZT), -1 * int(PAVT), int(SAZT), -1 * int(SAVT), int(SFZT), int(SFVT)]


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
