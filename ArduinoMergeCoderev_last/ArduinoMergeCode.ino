#include "DifferentialSteering.h"

const int serialDelay = 200;
int fPivYLimit = 32;
DifferentialSteering DiffSteer;
// Define Input Connection
//pwm pins: 	3, 5, 6, 9, 10, 11

//9-12 CH
//3-8 hbro
//0-1 comp, solenoid
//13 relay
//A0
#define CH1 9
#define CH2 10
#define CH5 11
#define CH6 12

#define comp_out 0
#define solenoid_out 1
#define relay 13

#define enA 3
#define in1 4
#define in2 5  
#define enB 6
#define in3 7
#define in4 8
// Integer to represent the value from the stick
int ch1Value;
int ch2Value;
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
//BMS Setup stuff
float temp = 0.0;
float r1 = 90900.0;
float r2 = 46200.0;
//bool compressorState = false;
const float batVolLim = 11.8;
const int batVoltMem = 5;  //Hvor mange spenninger bakover BMS'en skal huske
int loopNr = 0;            //For Bms
int time = 0;
bool compressor_state;

float battVoltage[batVoltMem];



bool checkBatt(float volt) {
  battVoltage[loopNr % batVoltMem] = volt;



  int i;
  bool volLim = false;
  float max;
  for (i = 0; i < batVoltMem; i++) {
    if (battVoltage[i] > max) {
      max = battVoltage[i];
    }
    /*Serial.print(i);
    Serial.print(": ");
    Serial.println(battVoltage[i]); */
  }

  if (max < batVolLim) {
    volLim = true;
  }

  //Serial.print(volLim);
  loopNr++;
  return volLim;
}

float getVoltage(int analogValue) {
  float temp = (analogValue * 5.0) / 1024.0;
  //Serial.println(temp / (r2/(r1+r2)));
  return temp / (r2 / (r1 + r2));
}

void shutOffCheck(float input_voltage) {
  if (checkBatt(input_voltage))  //Legger til mer sikkerhetsfaktor når man kjører
  {
    digitalWrite(relay, LOW);
  } else {
    // Serial.println("på");
    digitalWrite(relay, HIGH);
  }
}

void BMS(bool compressor, bool drive) {  //Takes in to arguments, compressor if compressor is on and drive if we currently use power on drivetrain.
  float inputVoltage = getVoltage(analogRead(0));

  //Serial.println(inputVoltage);


  if (!compressor)  //Hvis compressor er på sjekkes ikke spenning
  {
    if (drive) {
      shutOffCheck(inputVoltage + 0.7);  //Legger til mer sikkerhetsfaktor når man kjører
    } else {
      shutOffCheck(inputVoltage);
    }
  }
}


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  //Serial.begin(115200); //comment this out
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
  pinMode(relay, OUTPUT);
  pinMode(solenoid_out, OUTPUT);
  // Set initial rotation direction
  digitalWrite(in2, LOW);
  digitalWrite(in1, HIGH);
  digitalWrite(in4, LOW);
  digitalWrite(in3, HIGH);

  digitalWrite(relay, HIGH);
}

void loop() {
  int compressor_int = readChannel(CH6, 0, 10, 0);
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
  if (compressor_int > 4) {
    digitalWrite(comp_out, HIGH);
    compressor_state = true;
  } else {
    digitalWrite(comp_out, LOW);
    compressor_state = false;
  }
  if (solenoid_state > 4) {
    digitalWrite(solenoid_out, HIGH);
  } else {
    digitalWrite(solenoid_out, LOW);
  }


  //reverse rightmotor logic
  if (rightMotor < 0) {
    digitalWrite(in4, HIGH);
    digitalWrite(in3, LOW);
    rightMotor = -rightMotor;
  } else {
    digitalWrite(in4, LOW);
    digitalWrite(in3, HIGH);
  }
  //reverse leftmotor logic
  if (leftMotor < 0) {
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    leftMotor = -leftMotor;
  } else {
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
  }
  int pwmOutput1 = map(leftMotor, 0, 127, 0, 255); 
  int pwmOutput2 = map(rightMotor, 0, 127, 0, 255); 
  if (pwmOutput1 < 5){
    pwmOutput1 = 0;
  }
  if (pwmOutput2 < 5){
    pwmOutput2 = 0;
  }
  if (pwmOutput1 > 245){
    pwmOutput1 = 255;
  }
  if (pwmOutput2 > 245){
    pwmOutput2 = 255;
  }
  analogWrite(enA, pwmOutput1);                      // Send PWM signal to L298N Enable pin
  analogWrite(enB, pwmOutput2);
  //Serial.print("x value:");
  //Serial.println(XValue);
  //Serial.print("y value:");
  //Serial.println(YValue);
  //Serial.print("left motor:");
  //Serial.println(pwmOutput1);
  //Serial.print("right motor:");
  //Serial.println(pwmOutput2);

  //Serial.println(time);
  if (time % 10 == 0) {

    if (rightMotor > 4 || leftMotor > 4) {
      BMS(compressor_state, true);
    } else {
      BMS(compressor_state, false);
    }
    time = 0;
  }
  time++;
  delay(0.001);
}