import serial
import time

# Define the serial port for communicating with the Raspberry Pi Pico
serial_port = "/dev/ttyACM0"  # change this to your serial port
baudrate = 115200

# Create a Serial object for communication
ser = serial.Serial(serial_port, baudrate)

# Define a function to send a command to the Raspberry Pi Pico


def send_command(x, y, v, w):
    command = f"x:{x};y:{y};v:{v};w:{w}"
    ser.write(command.encode())
    print(f"Sending command: {command}")


# Continuously send commands
while True:
    send_command(0, 0, 1500, 0)
    time.sleep(0.25)