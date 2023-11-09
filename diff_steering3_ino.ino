
#include "DifferentialSteering.h"

const int serialDelay = 200;
int fPivYLimit = 32;
DifferentialSteering DiffSteer;
// Define Input Connection
#define CH1 3
#define CH2 11
#define CH6 8
#define CH5 12
#define comp_out 0
#define solenoid_out 1
// Integer to represent the value from the stick
int ch1Value;
int ch2Value;
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

#define enA 9
#define in1 6
#define in2 7

#define enB 10
#define in3 4
#define in4 5

void setup()
{

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200); //comment this out
    DiffSteer.begin(fPivYLimit);
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    pinMode(CH1, INPUT);
    pinMode(CH2, INPUT);
    pinMode(CH6, INPUT);
    pinMode(CH5, INPUT);

    pinMode(comp_out, OUTPUT);
    pinMode(solenoid_out, OUTPUT);
    // Set initial rotation direction
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in3, HIGH);
}

void loop()
{
    int compressor_state = readChannel(CH6, 0, 10, 0);
    int solenoid_state = readChannel(CH5, 0, 10, 0);
    int XValue = readChannel(CH1, -127, 127, 0);
    int YValue = readChannel(CH2, -127, 127, 0);
    int leftMotor = 0;
    int rightMotor = 0;
    DiffSteer.computeMotors(XValue, YValue);
    leftMotor = DiffSteer.computedLeftMotor();
    rightMotor = DiffSteer.computedRightMotor();
    //Serial.println("Differential | " + DiffSteer.toString());
    //Serial.print("comp state:");
    //Serial.println(compressor_state);
    //compressor
    if (compressor_state > 4){
      digitalWrite(comp_out, HIGH);
    } else {
      digitalWrite(comp_out, LOW);
    }
        if (solenoid_state > 4){
      digitalWrite(solenoid_out, HIGH);
    } else {
      digitalWrite(solenoid_out, LOW);
    }


    //reverse rightmotor logic
    if (rightMotor < 0){
      digitalWrite(in4, HIGH);
      digitalWrite(in3, LOW);
      rightMotor = -rightMotor;
    }
    else {
      digitalWrite(in4, LOW);
      digitalWrite(in3, HIGH);
    }
    //reverse leftmotor logic
    if (leftMotor < 0){
      digitalWrite(in2, HIGH);
      digitalWrite(in1, LOW);
      leftMotor = -leftMotor;
    }
    else {
      digitalWrite(in2, LOW);
      digitalWrite(in1, HIGH);
    }
  int pwmOutput1 = map(leftMotor, 0, 127, 0, 255); // Map the potentiometer value from 0 to 255
  int pwmOutput2 = map(rightMotor, 0, 127, 0, 255); // Map the potentiometer value from 0 to 255
  analogWrite(enA, pwmOutput1); // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutput2);
  //Serial.print("x value:");
  //Serial.println(XValue);
  //Serial.print("y value:");
  //Serial.println(YValue);
  //Serial.print("left motor:");
  //Serial.println(pwmOutput1);
  //Serial.print("right motor:");
  //Serial.println(pwmOutput2);
}