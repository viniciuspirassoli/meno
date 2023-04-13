import serial
import time


serialPort = "/dev/ttyACM0"  # change this to your serial port
baudrate = 115200
ser = serial.Serial(serialPort, baudrate)

def sendCommand(x, y, theta, print=False):
    ser.write("{:7.3f}".format(x) + "{:7.3f}".format(y) + "{:7.3f}".format(theta))
    if (print):
        print("sent: " + "{:7.3f}".format(x) + "{:7.3f}".format(y) + "{:7.3f}".format(theta))

while True:
    sendCommand(50, -50, 3, print=True)
    time.sleep(5)
    sendCommand(-50, 50, 3, print=True)
    time.sleep(5)
