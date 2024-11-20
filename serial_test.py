import serial
import time

# Configure serial port (adjust COM port as needed)
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

try:
  while True:
    # Create 8 bytes of test data (0-7 in this example)
    data = bytes([i for i in range(8)])
    
    # Send the data
    ser.write(data)
    
    # Wait 10ms
    time.sleep(0.05)

except KeyboardInterrupt:
  ser.close()
  print("Program terminated")