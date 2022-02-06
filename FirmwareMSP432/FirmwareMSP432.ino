///////////////////////////////////////////////////////////////////////////////////////////
// MSP432 Energia Communication System for ELEX4618 & ELEX4699
// Prepared by Craig Hennessey
// Last Edited: 2022-01-28 by Felix Serban
///////////////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>

// Constants for the ELEX4618 communication protocol TYPE field
enum {DIGITAL = 0, ANALOG, SERVO, GET, SET};

#define RGBLED_RED_PIN 39
#define RGBLED_GRN_PIN 38
#define RGBLED_BLU_PIN 37

// The MSP432 has 16 10-Bit ADC channels. The A_PIN array provides an easy way to map the 
// protocol CHANNEL integer to the A? pin
#define ANALOG_PINS 16
int A_PIN [] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15};

// Constants for the servo pins 
#define SERVO_PORT0 19
#define SERVO_PORT1 4
#define SERVO_PORT2 5
#define SERVO_PORT3 6

Servo myservo[4];

int type;
int channel;
int value;
int mode;
int response;
bool ledState;
unsigned long previousMillis = 0;
const long intervalMillisShort = 10;
const long intervalMillisLong = ((1000/intervalMillisShort)-1)*intervalMillisShort;
// ex. 1000/50 = 20; 20 - 1 = 19; 19 * 50 = 950. 
// Ensures that the period of the LED flash is one second.

#define BAUD_RATE 115200

void setup()
{
  // initialize serial port
  Serial.begin(BAUD_RATE);

  // initialize digital IO to Input 
  for (int digital_index = 1; digital_index <= 40; digital_index++)
  {
    pinMode(digital_index, INPUT_PULLUP);
  }

  // initialize MSP432 pushbuttons to Input (not on Boosterpack)
  pinMode(PUSH1, INPUT_PULLUP);
  pinMode(PUSH2, INPUT_PULLUP);

  // initialize MSP432 LED to ON (not on Boosterpack). Turn off RGB LED
  pinMode(RED_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  pinMode(RGBLED_RED_PIN, OUTPUT);
  pinMode(RGBLED_GRN_PIN, OUTPUT);
  pinMode(RGBLED_BLU_PIN, OUTPUT);
  digitalWrite(RGBLED_RED_PIN, LOW);
  digitalWrite(RGBLED_GRN_PIN, LOW);
  digitalWrite(RGBLED_BLU_PIN, LOW);

  // initialize servo pins to output
  pinMode(SERVO_PORT0, OUTPUT);
  pinMode(SERVO_PORT1, OUTPUT);
  pinMode(SERVO_PORT2, OUTPUT);
  pinMode(SERVO_PORT3, OUTPUT);

  // Attach servo pins to the servo objects
  myservo[0].attach(SERVO_PORT0);
  myservo[1].attach(SERVO_PORT1);
  myservo[2].attach(SERVO_PORT2);
  myservo[3].attach(SERVO_PORT3);

  Serial.print("\n////////////////////////////////////////////////////////////////////////////////////");
  Serial.print("\n// ELEX 4618 IO Communication for MSP432 V2.1 Student");
  Serial.print("\n// By: Felix Serban, 2022-01-28");
  Serial.print("\n// MSP432: Digital In/Out 1-40 on 4x 10 pin headers");
  Serial.print("\n// MSP432: Digital In 73 & 74 are PUSH1 and PUSH2 (MSP432)");
  Serial.print("\n// MSP432: Analog in A0 to A15 (0-15)");
  Serial.print("\n// MSP432: Analog out not supported");
  Serial.print("\n// MSP432: Servo 19,4,5,6 header (0-3)");
  Serial.print("\n// BoosterPack: Joystick (Analog 2,26), Accelerometer (Analog 23,24,25)");
  Serial.print("\n// BoosterPack: Buttons (Digital 32,33), LED (Digital 37,38,39)");
  Serial.print("\n// Protocol: DIRECTION (G/S) TYPE (0=D, 1=A, 2=S) CHANNEL VALUE");
  Serial.print("\n// Example: G 0 0, S 2 1 100");
  Serial.print("\n////////////////////////////////////////////////////////////////////////////////////\n");
}

void loop()
{
  // Heartbeat to indicate board is operational
  unsigned long currentMillis = millis();

  // Adapted from https://docs.arduino.cc/built-in-examples/digital/BlinkWithoutDelay
  if (ledState == HIGH && (currentMillis - previousMillis >= intervalMillisLong))
  {
    ledState = digitalRead(RED_LED);
    digitalWrite(RED_LED, !ledState);
    previousMillis = currentMillis;
  }
  else if (ledState == LOW && (currentMillis - previousMillis >= intervalMillisShort))
  {
    ledState = digitalRead(RED_LED);
    digitalWrite(RED_LED, !ledState);
    previousMillis = currentMillis;
  }

  // While there is data in the serial port buffer, continue to process
  while (Serial.available() > 0)
  {
    // Read the first character
    char ch = Serial.read();

    // If it's a COMMAND character (first character in ELEX4618 protocol) then move to next step
    if (ch == 'G' || ch == 'g' || ch == 'S' || ch == 's')
    {      
      // Read the space delimited next value as an integer (TYPE from protocol)
      type = Serial.parseInt();
      // Read the space delimited next value as an integer (CHANNEL from protocol)
      channel = Serial.parseInt();

      // If a SET command then read the space delimited next value as an integer (VALUE from protocol)
      if (ch == 'S' || ch == 's')
      {      
        value = Serial.parseInt();
        mode = SET;
      }
      else
      {
        mode = GET;
      }

      switch (type)
      {
        case DIGITAL:        
          if (mode == SET)
          {
            digitalWrite(channel, value);
          }
          else // GET
          {
            response = digitalRead(channel); 
          }
        break;

        case ANALOG:
          if (mode == SET)
          {
            break;
          }
          else // GET
          {
            response = analogRead(channel); 
          }
        break;

        case SERVO:
          if (mode == SET)
          {
            myservo[channel].write(value);
          }
          else // GET
          {
            response = myservo[channel].read();
          }
        break;

        default:
        break;
      }

      // Format and send response
      Serial.print ("A: ");
      Serial.print (type);
      Serial.print (" ");
      Serial.print (channel);
      Serial.print (" ");
      Serial.print (value);
      Serial.print (" ");
      Serial.print (response);
      Serial.print ("\n");
    }
  }
}
