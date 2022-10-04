// #include"PinDefinition.ino"
#include <MPU6050_light.h>
#include <Wire.h>
#include <time.h>
#include <Servo.h>

#define AUTOMATIC 0x1
#define MANUAL 0x0
#define KILO_HERTZ_TO_RPM 60000

#define pinBicycleMode 4
#define hallEffectSensor 7
// #define pinBypass 2
#define pinDownShift 9
#define pinUpShift 8
#define pinServo 10

#define NONE 0x0
#define UP 0x1
#define DOWN 0x2

const int gears[]=[500,780,1050,1290,1500,1780,2000,2300];
const int correctAmount = 200;
const int startingGear = 2;
const int maxGear = 9;

int Frequency = 60;           // recommended: between 50 (really slow cadence) and 90 (experienced cyclist of the sporty type)
int ToleranceHigherGear = 5;  // tolerance to move to smaller sprocket // recommended: between 2 and 10
int ToleranceLowerGear = 6;   // tolerance to move to larger sprocket // recommended: between 4 and 10
int ChangeDelayHeavy = 2000;  // recommended between 2500 and 5000
int ChangeDelayLight = 4700;  // recommended between 2500 and 5000
int MaxTurns = 5;             // recommended between 1 and 8
int currentGear = 0;
int action = NONE;
int pos = 0;

bool mode = AUTOMATIC;
bool gearChanged = false;

bool senseModeChanges = false;

const unsigned long debounceInterval = 250;
const unsigned long gearChangeInterval = 200;
// const unsigned long pitchReadingInterval = 10;
unsigned long timeModeChanges = 0;
unsigned long timeHallEffectReadingBefore = 0;
unsigned long timeGearChanged = 0;

float pitch = 0.0f;
float pedalSpeed = 0.0f;

// gyroscope
MPU6050 mpu(Wire);

Servo servo;


void setup() {
  Serial.begin(115200);
  servo.attach(pinServo);
  pinMode(pinBicycleMode, INPUT_PULLUP);
  pinMode(hallEffectSensor, INPUT_PULLUP);
  // pinMode(pinBypass, INPUT_PULLUP);
  pinMode(pinUpShift,INPUT_PULLUP);
  pinMode(pinDownShift,INPUT_PULLUP);
  attachInterrupt(hallEffectSensor,hallEffectReading, RISING)
  Wire.begin();
  mpu.begin();
  Serial.println(F("Calculating gyro offset, do not move MPU6050"));
  mpu.calcGyroOffsets();  // This does the calibration
}

void loop() {
  checkMode();
  bicycle();
  changeGear();
}

void checkMode() {
  bool modeNow = digitalRead(pinBicycleMode);

  // if there is changes in bicycle mode
  if (!senseModeChanges) {
    if (modeNow == !mode) {
      senseModeChanges = true;
      timeModeChanges = millis();
      mode=modeNow;
    }
  }
  // the mode in the next 250ms cant be changed to prevent bouncing, since before there is a change bicycle mode
  else {
    if(millis()-timeModeChanges>debounceInterval){
      senseModeChanges = false;
    }
  }
}

void bicycle(){
  // if mode true, means it is automatic
  if(mode){
    automatic();
  }
  // otherwise, it is manual
  else{
    manual();
  }
}

void automatic(){
  mpu.update();
  pitch=mpu.getAngleX();
  
  }
}

void manual(){

}

void changeGear(){
  // there is a changes in gear before hence check if the delay time has exceeded
  if(gearChanged){
    if(millis()-timeGearChanged>gearChangeInterval){
      gearChanged=false;
    }
  // there are no changes in gears before or the delay changes in gears has exceeded, therefore allow changes in gear again
  }else{
    if(action==UP){
      if(currentGear>1){
        currentGear++;
        pos = gears[currentGear];
        servo.writeMicroseconds(pos);
        gearChanged=true;
      }
    }
    else if (action==DOWN){
      if(currentGear<maxGear){
        currentGear--;
        pos = gears[currentGear];
        servo.writeMicroseconds(pos);
        gearChanged=true;
      }
    }
    else if(action==NONE){
      //do nothing
    }
  }
}

void resetSprocket(){
  pos = gears[startingGear];
  // the position should extended to ensure the chain fits to the gear
  int extendedPos = pos+correctAmount;
  servo.writeMicroseconds(extendedPos);
  delay(200);
  servo.writeMicroseconds(pos);
  delay(200);
  currentGear = startingGear;
}

// this function will be called when the there is a rising changes in hall effect sensor
void hallEffectReading(){
  unsigned long timeHallEffectReadingNow = millis();
  unsigned long intervalTwoHallEffectReading = timeHallEffectReadingNow-timeHallEffectReadingBefore;
  pedalSpeed = (1.0/intervalTwoHallEffectReading)*KILO_HERTZ_TO_RPM;
  timeHallEffectReadingBefore = timeHallEffectReadingNow;
}