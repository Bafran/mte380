#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Servo Configuration
#define NUM_SERVOS 6
#define SERVO_FREQ 50
#define SERVOMIN 150
#define SERVOMAX 600

// Global variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

const int SDA_Pin = 20;
const int SCL_Pin = 21;

int buffer[8] = { 0 };
int bufferIndex = 0;
bool packetStarted = false;
byte sum = 0;

void setup() {
  Serial.begin(9600);
  // Arduino joins I2C bus as slave with address 8
  Wire.begin(0x8);
  
  // Call receiveEvent function when data received                
  Wire.onReceive(receiveEvent);

  // Turn off 20k-50k ohm built-in pull up resistors at pins specified
  digitalWrite(SDA_Pin, LOW);
  digitalWrite(SCL_Pin, LOW);

  // Initialize servo driver
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  
  // Initialize all servos to middle position
  for(int i = 0; i < NUM_SERVOS; i++) {
    pwm.setPWM(i, 0, 375);  // Middle position
  }
  
  Serial.println("System initialized");
}

int mapServoValue(int input) {
    return map(input, 0, 180, 0, 450);
}
 
// Function that executes whenever data is received from master device
void receiveEvent(int howMany) {
  Serial.print("Received ");
  Serial.print(howMany);
  Serial.println(" bytes:");
  bufferIndex = 0;
  
  // Read and print each byte
  while (Wire.available()) {
    byte data = Wire.read();
    Serial.print(data);
    Serial.print(" ");
    Serial.println();

    // Check for start byte
    if (data == 0xFF) {
      bufferIndex = 0;
      sum = 0;
    }

    buffer[bufferIndex] = mapServoValue(data);
    bufferIndex++;

    if (bufferIndex > 0 && bufferIndex < 7) {
      sum += data;
    }

    // Validate checksum
    if (bufferIndex >= 7) {
     bufferIndex = 0;
     if (sum % 256 != data) {
      Serial.println("Checksum failed!");
     }
    }
  }
}

const int homePositions[NUM_SERVOS] = {150, 450, 150, 450, 150, 450};
const int directions[NUM_SERVOS] = {1, -1, 1, -1, 1, -1};  // -1 for -, 1 for +

void loop() {
  // Update servos every 1ms
  delay(100);

  for (int i = 0; i < NUM_SERVOS; i++) {
    int targetPosition = homePositions[i] + (directions[i] * buffer[i+1]);
    Serial.print("Writing ");
    Serial.print(targetPosition);
    Serial.print(" to motor ");
    Serial.print(i);
    Serial.println();
    pwm.setPWM(i, 0, targetPosition);
  }
}