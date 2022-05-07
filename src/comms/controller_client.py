import math
import numpy as np
import control.controller_translator as ct
import pygame as pg
import socket
import struct


def pilot_proc() -> None:
    """Sends controller input to Pico's Control over TCP connection.
    """
    hostname = 'localhost'
    port = 50004
    last_input = np.zeros(shape=(1, 1))
    started = False
    server_conn = False
    rcon_try_counter_max = 3
    rcon_try_count = 0
    pg.init()
    pg.joystick.init()
    js = pg.joystick.Joystick(0)
    js.init()
    print(str(js.get_numaxes()) + ' ' + str(js.get_numbuttons()) + ' ' + str(js.get_numhats()))
    ctc = ct.ControllerTranslator(joystick_drift_compensation=0.1, base_net_turn=10, base_net_strafe=-20, debug=True)
    print('Joystick set up')
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            s.connect((hostname, port))
            server_conn = True
            print('Connection to server established.')
        except ConnectionRefusedError as e:
            rcon_try_count += 1
            server_conn = False
            if rcon_try_count >= rcon_try_counter_max:
                started = False
            print(f'Connection to server failed... Reconnect attempt {rcon_try_count}/{rcon_try_counter_max}')
        while server_conn:
            try:
                data = s.recv(1024)
            except (ConnectionAbortedError, ConnectionResetError) as e:
                pilot_pipe_out.send(('gui', 'pilot', 'no_conn_socket'))
                print(f'Connection to server lost...')
                server_conn = False
                break
            if data == b'1':
                data = None
                if js.get_init():
                    control_in = np.zeros(shape=(1, js.get_numaxes()
                                                + js.get_numbuttons()
                                                + js.get_numhats()))
                    for i in range(js.get_numaxes()):
                        control_in.put(i, js.get_axis(i))
                    for i in range(js.get_numaxes(), js.get_numbuttons()):  # Buttons
                        control_in.put(i, js.get_button(i - js.get_numaxes()))

                    control_in.put((js.get_numaxes() + js.get_numbuttons()), js.get_hat(0))  # Hat
                    result = ctc.translate_to_maestro_controller(control_in)
                    # print(f'Result: {result}')
                pg.event.pump()
                if len(result) > 0:
                    last_input = struct.pack('>8b',
                                                  result[0],
                                                  result[1],
                                                  result[2],
                                                  result[3],
                                                  result[4],
                                                  result[5],
                                                  result[6],
                                                  result[7])
                    try:
                        s.sendall(last_input)
                    except (ConnectionAbortedError, ConnectionResetError) as e:
                        print(f'Connection to server lost...')
                        server_conn = False
                        break
                    result.clear()  # Clear input after sending latest
                else:  # Send previous input
                    try:
                        s.sendall(last_input)
                    except (ConnectionAbortedError, ConnectionResetError) as e:
                        print(f'Connection to server lost...')
                        server_conn = False
                        break
            else:
                print(f'Connection to server lost...')
                server_conn = False
                break
            if js.get_init():
                control_in = np.zeros(shape=(1, js.get_numaxes()
                                            + js.get_numbuttons()
                                            + js.get_numhats()))
                for i in range(js.get_numaxes()):
                    control_in.put(i, js.get_axis(i))
                for i in range(js.get_numaxes(), js.get_numbuttons()):  # Buttons
                    control_in.put(i, js.get_button(i - js.get_numaxes()))

                control_in.put((js.get_numaxes() + js.get_numbuttons()), js.get_hat(0))  # Hat
                result = ctc.translate_to_maestro_controller(control_in)
            pg.event.pump()
            if len(result) > 0:
                result.clear()


def run_client() -> None:
    """Test code using controller inputs directly. Don't run in other modules!"""
    pg.init()
    pg.joystick.init()
    js = pg.joystick.Joystick(0)
    js.init()
    print(str(js.get_numaxes()) + ' ' + str(js.get_numbuttons()) + ' ' + str(js.get_numhats()))
    ctc = ct.ControllerTranslator(joystick_drift_compensation=0.1, base_net_turn=10, base_net_strafe=-20, debug=True)
    print('Started Controller Client, sending input')
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
            result = ctc.translate_to_maestro_controller(control_in)
            print(f"PFZ: {result[0]} SFZ: {result[1]} SAZ: {result[2]} PAZ: {result[3]} PFV: {result[4]} "
                  f"PAV: {result[5]} SFV: {result[6]} SAV: {result[7]}")
        pg.event.pump()

if __name__ == '__main__':
    print('Starting Controller Client...')
    pilot_proc()
    #run_client()
