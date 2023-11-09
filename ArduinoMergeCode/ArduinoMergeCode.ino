
#include "DifferentialSteering.h"

const int serialDelay = 200;
int fPivYLimit = 32;
DifferentialSteering DiffSteer;
// Define Input Connection
#define CH1 3
#define CH2 11
#define CH5 8
#define CH6 12
#define comp_out 0
#define solenoid_out 1
#define relay 2
// Integer to represent the value from the stick
int ch1Value;
int ch2Value;
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
//BMS Setup stuff
float temp=0.0;
float r1=91300.0;
float r2=33100.0;
//bool compressorState = false;
const float batVolLim = 11.8;
const int batVoltMem = 5; //Hvor mange spenninger bakover BMS'en skal huske
int loopNr = 0; //For Bms
int time = 0;

float battVoltage[batVoltMem];

#define enA 9
#define in1 6
#define in2 7

#define enB 10
#define in3 4
#define in4 5

bool checkBatt(float volt) {
  battVoltage[loopNr % batVoltMem] = volt;

  int i;
  bool volLim = false;
  for (i = 0; i < batVoltMem; i++)
  {
    if (battVoltage[i] < batVolLim)
    {
      volLim = true;
    }
  }

  loopNr++;
  return volLim;
}

float getVoltage(int analogValue) {
  float temp = (analogValue * 5.0) / 1024.0;   
  return temp / (r2/(r1+r2));
}

void shutOffCheck(float input_voltage)
{
  if(checkBatt(input_voltage)) //Legger til mer sikkerhetsfaktor når man kjører
      {
        Serial.println("av");
        digitalWrite(relay, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else
      {
        digitalWrite(relay, HIGH);
        digitalWrite(LED_BUILTIN, LOW);
      }
}

void BMS(bool compressor, bool drive) {//Takes in to arguments, compressor if compressor is on and drive if we currently use power on drivetrain.
  float inputVoltage = getVoltage(analogRead(A0));

  Serial.println(inputVoltage);

  if(!compressor) //Hvis compressor er på sjekkes ikke spenning
  {
    if(drive)
    {
      shutOffCheck(inputVoltage + 0.3); //Legger til mer sikkerhetsfaktor når man kjører
    }
    else
    {
      shutOffCheck(inputVoltage);
    }
  }
}


void setup()
{

    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200); //comment this out
    DiffSteer.begin(fPivYLimit);
    pinMode(LED_BUILTIN, OUTPUT);
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

    digitalWrite(relay, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
}

void loop()
{
    int compressor_int = readChannel(CH6, 0, 10, 0);
    bool compressor_state = false;
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
    if (compressor_int > 4){
      digitalWrite(comp_out, HIGH);
      compressor_state = true;
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

  Serial.println(time);
  if (time % 10 == 0)
  {
    Serial.println(compressor_state);
    if (rightMotor > 4|| leftMotor > 4)
    {
        BMS(compressor_state, true);
    }
    else
    {
        BMS(compressor_state, false);
    }
  time = 0;  
  }
  time++;
  delay(0.001);
}