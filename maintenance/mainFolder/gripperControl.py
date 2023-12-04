import serial
import argparse
import sys
import time

def gripperControl(state = "open"):
    parser = argparse.ArgumentParser(description='A test program.')
    parser.add_argument("-p", "--usb_port", help="USB port.", default="/dev/ttyACM0")
    args = parser.parse_args()

    try:
        arduino = serial.Serial(args.usb_port, 9600)  # Adjust the baud rate accordingly
        print("Serial device connected!")
        print(state)
        if state == "open":
            angleset = 180
        if state == "close":
            angleset = 60
        if state == "imu":
            angleset = 115
        if state == "secretLid":
            angleset = 95
    
        angle = int(angleset)
        if 60 <= angle <= 180:
            arduino.write((str(angle)).encode('utf-8'))
    except serial.SerialException as e:
        print(f"Failed to connect to {args.usb_port}: {e}")
    
    time.sleep(2)

gripperControl(state="open")