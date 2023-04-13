import serial
import time
import pygame
import threading

# Define the serial port for communicating with the Raspberry Pi Pico
serial_port = "/dev/ttyACM0"  # change this to your serial port
baudrate = 115200

# Create a Serial object for communication
ser = serial.Serial(serial_port, baudrate)

# Initialize pygame and the joystick
pygame.init()
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Define a function to send a command to the Raspberry Pi Pico


def send_command(x, y, v, w):
    command = f"x:{x};y:{y};v:{v};w:{w}"
    ser.write(command.encode())

    print(f"Sending command: {command}")

# Define a function to read joystick input


def control_thread():
    while True:
        # Get the joystick input and calculate the v parameter
        pygame.event.pump()
        # use axis 1 for vertical movement
        joystick_axis = joystick.get_axis(1)
        # map joystick range (-1, 1) to command range (-5000, 5000)
        v = int(round(joystick_axis, 1) * -1000)
        # Store the v parameter in a global variable
        global v_global
        v_global = v
        # add a small delay to prevent sending commands too quickly
        time.sleep(0.1)

# Define a function to send commands using the v parameter from the control thread


def send_thread():
    while True:
        # Get the v parameter from the global variable
        global v_global
        v = v_global
        # Send the command with the v parameter
        send_command(0, 0, v, 0)
        # add a small delay to prevent sending commands too quickly
        time.sleep(0.2)


# Create a control thread and a send thread
control_thread = threading.Thread(target=control_thread)
send_thread = threading.Thread(target=send_thread)

# Start the control thread and the send thread
control_thread.start()
send_thread.start()