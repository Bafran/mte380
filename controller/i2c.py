from typing import List
from smbus2 import SMBus

class I2C:
  def __init__(self, bus_num: int = 1, addr:int = 0x8):
    """
    Args don't matter leave as defaults
    """
    self._bus = SMBus(bus_num)
    self._addr = addr
    self._header = 0xAA

    # Max/min angles to send to motors
    self._min_angle = 0
    self._max_angle = 90

  def send_data(self, data: List[int]) -> bool:
    """
    Data is a list of ints, must be 6 bytes.
    All values must be angles between 0 - 120 (arbitrary bounds chosen)
    """

    # Input len validation
    if len(data) != 6:
      print("Error: Data must be exactly 6 bytes")
      return False
    
    # Clamp angles
    data = [max(min(x, self._max_angle), self._min_angle) for x in data]

    # Add header
    msg = bytearray([self._header])
    msg.extend(data)

    # Calculate simple sum for CRC
    crc = sum(msg) % 256
    msg.append(crc)

    print(list(msg))

    # Send message
    try:
      self._bus.write_i2c_block_data(self._addr, msg[0], msg[1:])
      return True
    except:
      print("Error writing to I2C bus")
      return False

if __name__ == "__main__":
  # Test, smbus might need to be disabed to run
  bus = I2C()

  # Prints to console
  bus.send_data([-10, 0, 10, 60, 70, 130])