import numpy as np
import time
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State
from djipupper import HardwareInterface
from djipupper.IndividualConfig import SERIAL_PORT  # make the configs more consistent
from djipupper.Config import Configuration
from djipupper.Kinematics import four_legs_inverse_kinematics
import argparse

import datetime
import os
import msgpack
import socket
import pickle
import select

DIRECTORY = "logs/"
FILE_DESCRIPTOR = "walking"

auton_command = None
sock = None

def main(FLAGS):
    """Main program"""

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface.HardwareInterface(port=SERIAL_PORT)
    time.sleep(0.1)

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics)
    state = State(height=config.default_z_ref)
    print("Creating joystick listener...", end="")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    summarize_config(config)

    if FLAGS.log:
        today_string = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = os.path.join(
            DIRECTORY, FILE_DESCRIPTOR + "_" + today_string + ".csv"
        )
        log_file = open(filename, "w")
        hardware_interface.write_logfile_header(log_file)

    if FLAGS.zero:
        hardware_interface.set_joint_space_parameters(0, 4.0, 4.0)
        hardware_interface.set_actuator_postions(np.zeros((3,4)))
        input(
            "Do you REALLY want to calibrate? Press enter to continue or ctrl-c to quit."
        )
        print("Zeroing motors...", end="")
        hardware_interface.zero_motors()
        hardware_interface.set_max_current_from_file()
        print("Done.")
    else:
        print("Not zeroing motors!")

    if FLAGS.home:
        print("Homing motors...", end="", flush=True)
        hardware_interface.home_motors()        
        time.sleep(5)
        print("Done.")
        
    print("Waiting for L1 to activate robot.")

    last_loop = time.time()
    try:
        while True:
            if state.activation == 0:
                time.sleep(0.02)
                joystick_interface.set_color(config.ps4_deactivated_color)
                command = joystick_interface.get_command(state)
                if command.activate_event == 1:
                    print("Robot activated.")
                    joystick_interface.set_color(config.ps4_color)
                    time.sleep(0.1)
                    hardware_interface.serial_handle.reset_input_buffer()
                    time.sleep(0.1)
                    hardware_interface.activate()
                    time.sleep(0.1)
                    state.activation = 1
                elif command.home_event:
                    print("Homing motors manually")
                    hardware_interface.home_motors()
                    time.sleep(5)
            elif state.activation == 1:
                now = time.time()
                if FLAGS.log:
                    any_data = hardware_interface.log_incoming_data(log_file)
                    if any_data:
                      print(any_data['ts'])
                if now - last_loop >= config.dt:
                    command = joystick_interface.get_command(state)
                    if command.deactivate_event == 1:
                        print("Deactivating Robot")
                        print("Waiting for L1 to activate robot.")
                        time.sleep(0.1)
                        hardware_interface.deactivate()
                        time.sleep(0.1)
                        state.activation = 0
                        continue
                    if command.auton_mode:
                        update_auton_command()
                        if auton_command is not None:
                            command = auton_command
                    controller.run(state, command)
                    hardware_interface.set_cartesian_positions(
                        state.final_foot_locations
                    )
                    last_loop = now
    except KeyboardInterrupt:
        if FLAGS.log:
            print("Closing log file")
            log_file.close()

def update_auton_command():
    global sock
    if sock is None:
        print("Started Autonomous Sequence")
        server = socket.socket()
        server.bind(("localhost", 9999))
        server.listen(1)
        sock, addr = server.accept()
        sock.setblocking(False)
    
    rd, _, _ = select.select([sock], [], [], 0)
    if len(rd) > 0:
        next_command = sock.recv(1024)
        if next_command:
            global auton_command
            auton_command = pickle.loads(next_command)
            print("Got new command:", auton_command)

def summarize_config(config):
    # Print summary of configuration to console for tuning purposes
    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("default height: ", config.default_z_ref)
    print("x shift: ", config.x_shift)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--zero", help="zero the motors", action="store_true")
    parser.add_argument("--log", help="log pupper data to file", action="store_true")
    parser.add_argument("--home", help="home the motors (moves the legs)", action="store_true")
    FLAGS = parser.parse_args()
    main(FLAGS)
