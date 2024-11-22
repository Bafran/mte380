from typing import List
import serial
import time

BAUDRATE = 115200

class MotorSerial:
  def __init__(self, port='COM5'):
    self._port = port
    self._ser: serial.Serial = serial.Serial(port, BAUDRATE, timeout=1)
    time.sleep(2)

  def _calculate_checksum(self, data):
    return sum(data) % 256

  def send(self, angles: List[int]) -> None:
    checksum = self._calculate_checksum(angles)

    try:
      # Calculate checksum
      # checksum = self._calculate_checksum(angles)
      
      # Construct full packet
      packet = [0xFF] + [a+100 for a in angles]
      
      # Send the packet
      self._ser.write(packet)

      print(f"packet: {[p-100 for p in packet]}")
      
    except:
      print("Serial connection failed")