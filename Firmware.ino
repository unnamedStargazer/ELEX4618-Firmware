///////////////////////////////////////////////////////////////////////////////////////////
// MSP432 Energia Communication System for ELEX4618 & ELEX4699
// Prepared by: Craig Hennessey
// Last Edited: Oct 30, 2017
///////////////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>

enum {DIGITAL = 0, ANALOG, SERVO};

#define ANALOG_PINS 16
int A_PIN [] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

#define SERVO_PORT0 19
#define SERVO_PORT1 4
#define SERVO_PORT2 5
#define SERVO_PORT3 6

Servo myservo[4];

int type;
int channel;
int value;

#define BAUD_RATE 115200

void setup()
{
  // initialize serial
  Serial.begin(BAUD_RATE);
  Serial.setTimeout(10);

  // initialize digital IO to Input
  for (int digital_index = 1; digital_index <= 40; digital_index++)
  {
    pinMode(digital_index, INPUT_PULLUP);
  }
  
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  // initialize LED display
  pinMode(RED_LED, OUTPUT);

  // initialize servos
  pinMode(SERVO_PORT0, OUTPUT);
  pinMode(SERVO_PORT1, OUTPUT);
  pinMode(SERVO_PORT2, OUTPUT);
  pinMode(SERVO_PORT3, OUTPUT);
  
  myservo[0].attach(SERVO_PORT0);
  myservo[1].attach(SERVO_PORT1);
  myservo[2].attach(SERVO_PORT2);
  myservo[3].attach(SERVO_PORT3);

  Serial.print("\n////////////////////////////////////////////////////////////////////////////////////");
  Serial.print("\n// ELEX 4618 IO Communication for MSP433 V2.0 Student");
  Serial.print("\n// By: STUDENT NAME, DATE");
  Serial.print("\n// MSP432: Digital In/Out 1-40 on 4 headers");
  Serial.print("\n// MSP432: Digital In 41 & 42 are PUSH1 and PUSH2");
  Serial.print("\n// MSP432: Analog in A0 to A15 (0-15)");
  Serial.print("\n// MSP432: Analog out not supported");
  Serial.print("\n// MSP432: Servo 19,4,5,6 header (0-3)");
  Serial.print("\n// BoosterPack (432): Accelerometer (A 11,13,14), Joystick (A 9,15), Buttons (D 32,33), LED (37,38,39)");
  Serial.print("\n// Protocol: DIRECTION (G/S) TYPE (0=D, 1=A, 2=S) CHANNEL VALUE");
  Serial.print("\n// Example: G 0 0, S 2 1 100");
  Serial.print("\n////////////////////////////////////////////////////////////////////////////////////\n");
}

void loop()
{
  /////////////////////////////////////////
  // TODO: Flash LED ON/OFF
  /////////////////////////////////////////
  
  while (Serial.available() > 0)
  {
    char ch = Serial.read();

    if (ch == 'G' || ch == 'g' || ch == 'S' || ch == 's')
    {      
      type = Serial.parseInt();
      channel = Serial.parseInt();

      // Only for SET
      if (ch == 'S' || ch == 's')
      {      
        value = Serial.parseInt();
      }

      /////////////////////////////////////////
      // TODO: Get / Set Digital
      /////////////////////////////////////////
      
      /////////////////////////////////////////
      // TODO: Get / Set Analog
      /////////////////////////////////////////
      
      /////////////////////////////////////////
      // TODO: Get / Set Servo
      /////////////////////////////////////////

      Serial.print ("A ");
      Serial.print (type);
      Serial.print (" ");
      Serial.print (channel);
      Serial.print (" ");
      Serial.print (value);
      Serial.print ("\n");
    }
  }
}

