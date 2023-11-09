float input_voltage = 0.0;
float temp=0.0;
float r1=91300.0;
float r2=33100.0;
bool compressorState = false;
const float batVolLim = 11.8;
const int batVoltMem = 5;
int loopNr = 0; //For Bms

//Pins
const int reelayPin = 2;

float battVoltage[batVoltMem];


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

void shutOffCheck(bool input_voltage)
{
  if(checkBatt(input_voltage)) //Legger til mer sikkerhetsfaktor når man kjører
      {
        digitalWrite(reelayPin, LOW);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else
      {
       digitalWrite(reelayPin, HIGH);
       digitalWrite(LED_BUILTIN, LOW);
     }
}

void BMS(bool compressor, bool drive) {//Takes in to arguments, compressor if compressor is on and drive if we currently use power on drivetrain.
  float inputVoltage = getVoltage(analogRead(A0));

  if(!compressor) //Hvis compressor er på sjekkes ikke spenning
  {
    if(drive)
    {
      shutOffCheck(input_voltage + 0.3); //Legger til mer sikkerhetsfaktor når man kjører
    }
    else
    {
      shutOffCheck(input_voltage);
    }
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);     //  opens serial port, sets data rate to 9600 bps
  pinMode(reelayPin, OUTPUT);     //Truns Robot off if low
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(3, INPUT) //Input fra kontroller om compressor skal være på eller ikke
}

void loop() {
  //Get state if compressor is on
  compressorState = digitalRead(3);

  BMS(compressorState, driveState); //BMS, takes maximum O(batVolMem) time
  

  
  delay(1000);
}
