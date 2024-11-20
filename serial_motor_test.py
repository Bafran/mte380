import serial
import time

# Configure serial port (adjust COM port as needed)
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Wait for Arduino to reset

def calculate_checksum(data):
  return sum(data) % 256

try:

  counter = 0
  increment = 0

  while True:
    # Create packet: Start byte (0xFF) + 6 servo positions + checksum
    angle = 0 + increment
    servo_positions = [angle, angle, angle, angle, angle, angle]  # Example positions (0-180 degrees)
    
    # Calculate checksum
    checksum = calculate_checksum(servo_positions)
    
    # Construct full packet
    packet = bytes([0xFF] + servo_positions + [checksum])
    
    # Send the packet
    ser.write(packet)
    
    # Wait 10ms
    time.sleep(0.05)

    if counter % int(1/0.10) == 0:
      increment += 1
      counter = 0

    if increment > 30:
      increment = 0

    

except KeyboardInterrupt:
  ser.close()
  print("Program terminated")