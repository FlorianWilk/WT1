import serial
import time
ser = serial.Serial('/dev/ttyACM0',115200)  # open serial port
print(ser.name)         # check which port was really used
time.sleep(3)
ser.write(b"s\r\n")     # write a string
time.sleep(4)
ser.write(b"x\r\n")
ser.flush()
ser.close()             # close port
