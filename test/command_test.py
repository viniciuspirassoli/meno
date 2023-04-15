import serial
import time

set = False

start_time = time.time()

serialPort = "COM12"  # change this to your serial port
baudrate = 115200
ser = serial.Serial(serialPort, baudrate)

def sendCommand(v, w, theta):
    ser.write(bytes("{:8.3f}".format(v) + "{:8.3f}".format(w) + "{:8.3f}".format(theta), encoding='utf-8'))
    print("sent: " + "{:8.3f}".format(v) + "{:8.3f}".format(w) + "{:8.3f}".format(theta))

while True:
    try:
        if ser.in_waiting > 0:
            # Read from serial port
            data = ser.readline().decode('utf-8').rstrip()
            print(data)

        # Check if 5 seconds have elapsed
        elapsed_time = time.time() - start_time
        if elapsed_time >= 5:
            # Write to serial port
            if set:
                sendCommand(50, -50, 0)
            else:
                sendCommand(-50, 50, 0)
                
            set = not set
            # Reset start time
            start_time = time.time()

    except KeyboardInterrupt:
        sendCommand(0, 0, 0)
        ser.close()
        break
