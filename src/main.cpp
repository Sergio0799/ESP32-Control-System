/* Include necessary libraries */
// Arduino & Serial communication:
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
// Sensors (Ultrasonic, Compass, BMI):
#include <NewPing.h>
//#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <BMI160Gen.h>
//#include <CurieIMU.h>
// Motors & Servos:
#include <L298N.h>
#include <ESP32Servo.h>

// PS5 controller testing:
#include <ps5Controller.h>

// /* Library and variable initialization for degree calculations from BMI160 */
// #include <MadgwickAHRS.h>
// Madgwick filter;
// unsigned long microsPerReading, microsPrevious;


/* Assign pin names, based on GPIO number: */
// Servos
const int shaftPWM = 16;
const int scraperPWM = 17;
// Motor Drivers
const int trayMDriverIn2 = 15;
const int trayMDriverIn1 = 2;
const int trackMDriverIn4 = 25;
const int trackMDriverIn3 = 26;
const int trackMDriverIn2 = 27;
const int trackMDriverIn1 = 14;
const int trayEnable = 4;
const int track1Enable = 13;
const int track2Enable = 12;
// Ultrasonic Sensors
const int frontUltrasonicEcho = 34;
const int frontUltrasonicTrig = 32;
const int binUltrasonicEcho = 35;
const int binUltrasonicTrig = 33;
// UART Communication w/ RPi
const int espRX = 3;
const int espTX = 1;
//const int espRX = 16;
//const int espTX = 17;
// I2C Communication w/ Compass
const int I2CSerialClock = 22;
const int CompassData = 21;
// SPI Communication w/ BMI
const int BMIChipSelect = 5;
const int BMISerialClock = 18;
const int BMIMISO = 19;
const int BMIMOSI = 23;

/* Initialize necessary structures (from libraries): */
// Servos
Servo shaftServo;
Servo handServo;
ESP32PWM pwm;
// Motor Drivers
L298N rightMotor(track1Enable, trackMDriverIn1, trackMDriverIn2);
L298N leftMotor(track2Enable, trackMDriverIn3, trackMDriverIn4);
L298N trayMotor(trayEnable, trayMDriverIn1, trayMDriverIn2);
// Ultrasonic Sensors
NewPing sonar(frontUltrasonicTrig, frontUltrasonicEcho, 200);
NewPing binFill(binUltrasonicTrig, binUltrasonicEcho, 40);
// UART Serial Comm w/ RPi
HardwareSerial MySeri(2);
// Compass
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  // put your setup code here, to run once:
  // Allow allocation of all timers
	ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);
  // Standard 50hz servos
  shaftServo.setPeriodHertz(50);
  handServo.setPeriodHertz(50);

  ps5.begin("F0:2F:4B:0F:78:59");
  // Initialize the I2C library
  Wire.begin();
  
  // Initialize serial communication with a 9600 baud rate, through microUSB
  Serial.begin(9600);
  while (!Serial);  // wait for the serial port to open
  Serial.println("USB Serial Initialized"); // output to console when opened

  // Initialize serial communication to RPi (9600 bps, 8 bits, No Parity, 1 Stop Bit)
  //MySeri.begin(9600, SERIAL_8N1, espRX, espTX);
  //MySeri.begin(9600);
  //MySeri.setDebugOutput(true);
  //while(!MySeri); // wait for serial port to open
  // Serial.println("Serial Initialized"); // output to console when opened
  
  // Initialize the SPI library
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV4);  //divide the clock by 4

  
  

  // Set pins mode for each output
  pinMode(shaftPWM, OUTPUT);
  pinMode(scraperPWM, OUTPUT);
  pinMode(trayMDriverIn2, OUTPUT);
  pinMode(trayMDriverIn1, OUTPUT);
  pinMode(trayEnable, OUTPUT);
  pinMode(trackMDriverIn4, OUTPUT);
  pinMode(trackMDriverIn3, OUTPUT);
  pinMode(trackMDriverIn2, OUTPUT);
  pinMode(trackMDriverIn1, OUTPUT);
  pinMode(track1Enable, OUTPUT);
  pinMode(track2Enable, OUTPUT);
  pinMode(frontUltrasonicTrig, OUTPUT);
  pinMode(binUltrasonicTrig, OUTPUT);
  pinMode(BMIChipSelect, OUTPUT);
  // Set pins mode for each input
  pinMode(frontUltrasonicEcho, INPUT);
  pinMode(binUltrasonicEcho, INPUT);
  // Serial.println("Pins Defined"); // output to console that pins were defined

  // Print error if magnetometor not initialized
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    // Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }

  // Give sensors & serial communication time to initialize
  delay(100);
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;

  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;

  return g;
}

int compassDegree() {
  // Get a new sensor event
  sensors_event_t event;
  mag.getEvent(&event);

  // Read heading based on the x & y axis
  float heading = atan2(event.magnetic.y, event.magnetic.x);

  // Add 'Declination Angle', corrects for error in local magnetic field
  // MUST BE UPDATED MANUALLY BASED ON LOCATIION: magnetic-declination.com
  float declinationAngle = 0.0573;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  
  // Convert radians to degrees
  float headingDegrees = heading * 180/PI;

  // Return degree currently facing
  return headingDegrees;
}

void turn(int degree) {
  // Initialize motor driver variable & assign arbitrarily (will be reassigned)
  L298N forwardMotor = trayMotor;
  L298N reverseMotor = trayMotor;

  // Get degree robot is initially facing
  int initDegree = compassDegree();
  // Print message to computer (troubleshooting)
  // Serial.print("Initial degree of robot: ");
  // Serial.println(initDegree);

  // Assign which motor will move forward/reverse based on turning
  if (degree < 0) {
    forwardMotor = leftMotor;
    reverseMotor = rightMotor;
  }
  else if (degree > 0) {
    forwardMotor = rightMotor;
    reverseMotor = leftMotor;
  }
  else {
    
  }

  unsigned short dutyCycle = 50;
  int currentDegree, halfwayDegree = degree / 2;
  // Turn robot at incremental speed (no sudden burst of movement)
  for (; dutyCycle < 200; dutyCycle = dutyCycle + 5) {
    forwardMotor.setSpeed(dutyCycle);
    reverseMotor.setSpeed(dutyCycle);
    // Tracks must be rotating in opposite directions for static turn
    forwardMotor.forward();
    reverseMotor.backward();
    // Print message to computer (troubleshooting)
    // Serial.print("Turn with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(forwardMotor.getSpeed());
    // Constantly check current degree reading and stop accelerating when halfway through turning
    currentDegree = compassDegree();
    if ((initDegree - currentDegree) == halfwayDegree) {
      // Print message to computer (troubleshooting)
      // Serial.print("Slowing turn at degree change: ");
      // Serial.println(initDegree - currentDegree);
      break;
    }
    delay(100);
  }

  // Keep turning even when max speed is reached, until reaching halfway point of turn
  while ((initDegree - currentDegree) != halfwayDegree) {
    currentDegree = compassDegree();
    // Print message to computer (troubleshooting)
    // Serial.print("Current degree: ");
    // Serial.println(currentDegree);
  }
  // Print message to computer (troubleshooting)
  // Serial.print("Slowing turn at degree change: ");
  // Serial.println(initDegree - currentDegree);

  // Decrease speed of turn after halfway (no sudden stop)
  for (; dutyCycle > 60; dutyCycle = dutyCycle - 5) {
    forwardMotor.setSpeed(dutyCycle);
    reverseMotor.setSpeed(dutyCycle);
    // Tracks must be rotating in opposite directions for static turn
    forwardMotor.forward();
    reverseMotor.backward();
    // Print message to computer (troubleshooting)
    // Serial.print("Turn with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(forwardMotor.getSpeed());
    // Constantly check current degree reading and stop when desired degree change is reached
    currentDegree = compassDegree();
    if ((initDegree - currentDegree) == degree) {
      // Print message to computer (troubleshooting)
      // Serial.print("Slowing turn at degree change: ");
      // Serial.println(initDegree - currentDegree);
      break;
    }
    delay(100);
  }

  // Keep turning even when low speed is reached, until reaching desired degree change
  while ((initDegree - currentDegree) != degree) {
    currentDegree = compassDegree();
    // Print message to computer (troubleshooting)
    // Serial.print("Current degree: ");
    // Serial.println(currentDegree);
  }
  // Print message to computer (troubleshooting)
  // Serial.print("Stopping turn at degree change: ");
  // Serial.println(initDegree - currentDegree);

  // Fully stop turning, should be facing object now
  forwardMotor.stop();
  reverseMotor.stop();
}

void openTray() {
  unsigned short dutyCycle = 50;
  // Move DC motor forward with increasing speed
  for (; dutyCycle < 250; dutyCycle = dutyCycle + 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.backward();
    // Serial.print("Opening with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(500);
  }
  // Move DC motor forward with decreasing speed
  for (; dutyCycle > 80; dutyCycle = dutyCycle - 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.backward();
    // Serial.print("Opening with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(500);
  }
}

void closeTray() {
  unsigned short dutyCycle = 50;
  // Move DC motor forward with increasing speed
  for (; dutyCycle < 250; dutyCycle = dutyCycle + 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.forward();
    // Serial.print("Clowing with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(500);
  }
  // Move DC motor forward with decreasing speed
  for (; dutyCycle > 80; dutyCycle = dutyCycle - 5) {
    trayMotor.setSpeed(dutyCycle);
    trayMotor.forward();
    // Serial.print("Closing with duty cycle: ");
    // Serial.print(dutyCycle);
    // Serial.print(" with function reading of: ");
    // Serial.println(trayMotor.getSpeed());
    delay(500);
  }
}

int bigMinUs = 365; // 365 for MG996R
int bigMaxUs = 2470; // 2460-2480 for MG996R
// int smallMinUs = 500; // 500 for SG90
// int smallMaxUs = 2400; // 2400 for SG90

void lowerShaft() {
  int pos = 0;
  shaftServo.attach(4, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 160; pos >= 50; pos -= 1) { // sweep from 180 degrees to 0 degrees
		shaftServo.write(pos);
    //leftArmServo.write((160-pos)+50);
		delay(20);
	}

  // Serial.print("Shaft servo value: ");
  // Serial.println(shaftServo.read());

  shaftServo.detach();
  pwm.detachPin(27);
}

void raiseShaft() {
  int pos = 0;
  shaftServo.attach(4, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 50; pos <= 160; pos += 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		shaftServo.write(pos);
    //leftArmServo.write((160-pos)+50);
		delay(20);             // waits 20ms for the servo to reach the position
	}

  // Serial.print("Shaft servo value: ");
  // Serial.println(shaftServo.read());

  shaftServo.detach();
  pwm.detachPin(27);
}

void closeHands() {
  int pos = 0;
  handServo.attach(2, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 170; pos >= 10; pos -= 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		handServo.write(pos);
		delay(10);             // waits 20ms for the servo to reach the position
	}

  // Serial.print("Hand servo value: ");
  // Serial.println(handServo.read());

  handServo.detach();
  pwm.detachPin(27);
}

void openHands() {
  int pos = 0;
  handServo.attach(2, bigMinUs, bigMaxUs);
  pwm.attachPin(27, 10000);//10khz

  for (pos = 10; pos <= 170; pos += 1) { // sweep from 0 degrees to 180 degrees
		// in steps of 1 degree
		handServo.write(pos);
		delay(10);             // waits 20ms for the servo to reach the position
	}

  // Serial.print("Hand servo value: ");
  // Serial.println(handServo.read());

  handServo.detach();
  pwm.detachPin(27);
}



void loop() {
  // put your main code here, to run repeatedly:
  char data = 0;
  bool error = false;
  
  // Read instruction from RPi when available
  if (MySeri.available()) {
    data = MySeri.read();
  }

  // Production code (not testing)
  if (!ps5.isConnected()) { 
    
    // Only check what action to take if there was an instruction given
    if (data != 0) {
      // Determine what actions to take based on received instruction
      switch(data) {
        case 'A': { // Turn to a specific angle
          // Read degree of turn as int
          int degrees = MySeri.parseInt();

          // Print message to computer (troubleshooting)
          // Serial.print("Turning to degree: ");
          // Serial.println(degrees);

          //delay(1000);  
          turn(degrees);
          // Send confirmation code to RPi
          MySeri.write('7');
          break;
        }

        case 'B': {// 
          break;
        }

        case 'C': { // Object centered, continue forward
          // Initialize necessary local variables for case
          unsigned short rightCycle = rightMotor.getSpeed();
          unsigned short leftCycle = leftMotor.getSpeed();
          unsigned short dutyCycle = 200;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Check if tracks are moving forward at same speed (shouldn't be)
          if (rightCycle != leftCycle) {
            // Set both motors to the lowest of the two's speeds
            if (rightCycle > leftCycle) {
              dutyCycle = leftCycle;
            }
            else {
              dutyCycle = rightCycle;
            }
          }

          // Move DC motors forward with increasing speed
          for (; dutyCycle < 250; dutyCycle = dutyCycle + 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
            delay(50);
          }
            
          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'D': { // Decelerate forward
          // Initialize necessary local variables for case
          unsigned short rightCycle = rightMotor.getSpeed();
          unsigned short leftCycle = leftMotor.getSpeed();
          unsigned short dutyCycle = 200;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Check if tracks are moving forward at same speed
          if (rightCycle != leftCycle) {
            // Set both motors to the lowest of the two's speeds
            if (rightCycle > leftCycle) {
              dutyCycle = leftCycle;
            }
            else {
              dutyCycle = rightCycle;
            }
          }

          // Move DC motors forward with decreasing speed
          for (; dutyCycle > 50; dutyCycle = dutyCycle - 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 15) {
              MySeri.write('7');
              rightMotor.stop();
              leftMotor.stop();
              newInstruction = true;
              break;
            }
            delay(50);
          }

          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 15) {
              MySeri.write('7');
              rightMotor.stop();
              leftMotor.stop();
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'E': { // Emergency stop
          rightMotor.stop();
          leftMotor.stop();
          error = true;
          break;
        }

        case 'F': { // Accelerate forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = 50;
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Move DC motors forward with increasing speed
          for (; dutyCycle < 250; dutyCycle = dutyCycle + 10) {
            rightMotor.setSpeed(dutyCycle);
            leftMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Forward with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" with function reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
            delay(50);
          }
            
          while (!newInstruction) {
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(25);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
          }

          break;
        }

        case 'L': { // Turn slightly left while moving forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = leftMotor.getSpeed();
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Slowly decelerate left motor to turn slightly left
          for (; dutyCycle > 25; dutyCycle = dutyCycle - 5) {
            leftMotor.setSpeed(dutyCycle);
            leftMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Left with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" While right motor reading of: ");
            // Serial.println(rightMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
            delay(50);
          }

          // If finish the loop and still not centered there is an error
          if (!newInstruction) {
            error = true;
          }
          break;
        }

        case 'O': {// Everything ok bro?
          //MySe
          break;
        }

        case 'P': { // Engage pickup function
          // Serial.println("Pickup function here");
          // Open hands while still above bin then lower them (around the object)
          openHands();
          lowerShaft();
          // Close hands under object and tilt it into the bin
          closeHands();
          raiseShaft();
          // Tell RPi when finished picking up
          MySeri.write('7');
          break;
        }

        case 'R': { // Turn slightly right while moving forward
          // Initialize necessary local variables for case
          unsigned short dutyCycle = rightMotor.getSpeed();
          unsigned long ping = 0;
          unsigned long distance = 0;
          boolean newInstruction = false;

          // Slowly decelerate left motor to turn slightly left
          for (; dutyCycle > 25; dutyCycle = dutyCycle - 5) {
            rightMotor.setSpeed(dutyCycle);
            rightMotor.forward();
            // Print message to computer (troubleshooting)
            // Serial.print("Right with duty cycle: ");
            // Serial.print(dutyCycle);
            // Serial.print(" While left motor reading of: ");
            // Serial.println(leftMotor.getSpeed());
            // Constantly check if another instruction is sent, if so break out of case
            if (MySeri.available()) {
              data = MySeri.read();
              newInstruction = true;
              break;
            }
            // Constanly check front ultrasonic sensor
            ping = sonar.ping_median(20);
            distance = sonar.convert_cm(ping);
            // Print message to computer (troubleshooting)
            // Serial.print("Sonar reading: ");
            // Serial.println(distance);
            // If object is close, break out of case
            if (distance > 0 && distance < 50) {
              MySeri.write('7');
              newInstruction = true;
              break;
            }
            delay(50);
          }

          // If finish the loop and still not centered there is an error
          if (!newInstruction) {
            error = true;
          }
          break;
        }

        case 'T': { // Do a 180

          break;
        }

        default: {
          MySeri.println("Serial communication error");
          // // Serial.println("Serial communication error");
          error = true;
          break;
        }
      }
    } else Serial.println("Waiting for RPI serial comms");

  } //End of RPI code line
  else if(ps5.isConnected()) { //Manuel testing
    if (ps5.Right()) Serial.println("Right Button");
    if (ps5.Down()) Serial.println("Down Button");
    if (ps5.Up()) Serial.println("Up Button");
    if (ps5.Left()) Serial.println("Left Button");

    if (ps5.Square()) Serial.println("Square Button");
    if (ps5.Cross()) Serial.println("Cross Button");
    if (ps5.Circle()) Serial.println("Circle Button");
    if (ps5.Triangle()) Serial.println("Triangle Button");

    if (ps5.UpRight()) Serial.println("Up Right");
    if (ps5.DownRight()) Serial.println("Down Right");
    if (ps5.UpLeft()) Serial.println("Up Left");
    if (ps5.DownLeft()) Serial.println("Down Left");

    if (ps5.L1()) Serial.println("L1 Button");
    if (ps5.R1()) Serial.println("R1 Button");

    if (ps5.Share()) Serial.println("Share Button");
    if (ps5.Options()) Serial.println("Options Button");
    if (ps5.L3()) Serial.println("L3 Button");
    if (ps5.R3()) Serial.println("R3 Button");

    if (ps5.PSButton()) Serial.println("PS Button");
    
    if (ps5.Touchpad()) Serial.println("Touch Pad Button");

    if (ps5.L2()) {
      Serial.printf("L2 button at %d\n", ps5.L2Value());
      leftMotor.setSpeed(ps5.L2Value());
      leftMotor.backward();
      rightMotor.setSpeed(ps5.L2Value());
      rightMotor.forward();
      
    } else {
      rightMotor.stop();
      leftMotor.stop();
    }
    if (ps5.R2()) {
      Serial.printf("R2 button at %d\n", ps5.R2Value());
    }

    // if (ps5.LStickX()) {
    //   Serial.printf("Left Stick x at %d\n", ps5.LStickX());
    // }
    // if (ps5.LStickY()) {
    //   Serial.printf("Left Stick y at %d\n", ps5.LStickY());
    // }
    // if (ps5.RStickX()) {
    //   Serial.printf("Right Stick x at %d\n", ps5.RStickX());
    // }
    // if (ps5.RStickY()) {
    //   Serial.printf("Right Stick y at %d\n", ps5.RStickY());
    // }

    Serial.println();
    // This delay is to make the output more human readable
    // Remove it when you're not trying to see the output
    delay(50);
  }

  // If there was an error detected then print msg to computer
  if (error) {
    // Serial.println("Something went wrong!");
  }
  //delay(1000);
}