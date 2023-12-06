import serial
import argparse
import sys
import time


def gripperControl(state="open"):
    # Here and args values if made to parse specification
    # for the connection to the seeduino
    parser = argparse.ArgumentParser(
        description='A test program.')
    parser.add_argument("-p", "--usb_port",
                        help="USB port.", default="/dev/ttyACM0")
    args = parser.parse_args()

    # The code tries to connect to the seeduino with the args usb
    # then it reads the given state ans writes it to the chip
    # with encode of 'utf-8' style
    try:
        # Adjust the baud rate accordingly
        arduino = serial.Serial(args.usb_port, 9600)
        print("Serial device connected!")
        print(state)
        if state == "open":
            angleset = 180
        if state == "close":
            angleset = 60
        if state == "imu":
            angleset = 115
        if state == "secretLid":
            angleset = 90

        angle = int(angleset)
        if 60 <= angle <= 180:
            arduino.write((str(angle)).encode('utf-8'))
    # This gives and error if the connection was not established
    except serial.SerialException as e:
        print(f"Failed to connect to {args.usb_port}: {e}")
    # It sleeps for two seconds to make sure the gripper
    # is in the wanted state before moving the robot further
    time.sleep(2)
