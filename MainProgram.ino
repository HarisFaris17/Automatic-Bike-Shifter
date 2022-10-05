// #include"PinDefinition.ino"
#include <MPU6050_light.h>
#include <Wire.h>
#include <time.h>
#include <Servo.h>

#define AUTOMATIC_GRADIENT 0x0
#define AUTOMATIC_SPEED 0X1
#define MANUAL 0x2

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

const int gears[] = { 500, 780, 1050, 1290, 1500, 1780, 2000, 2300, 2600 };
const int correctAmount = 200;
const int startingGear = 2;
const int maxGear = 9;

int frequencyTarget = 60;     // recommended: between 50 (really slow cadence) and 90 (experienced cyclist of the sporty type)
int toleranceHigherGear = 5;  // tolerance to move to smaller sprocket // recommended: between 2 and 10
int toleranceLowerGear = 6;   // tolerance to move to larger sprocket // recommended: between 4 and 10
int changeDelayHeavy = 2000;  // recommended between 2500 and 5000
int changeDelayLight = 4700;  // recommended between 2500 and 5000
// int naxTurns = 5;             // recommended between 1 and 8
int currentGear = 0;
int action = NONE;
int pos = 0;

int mode = AUTOMATIC_GRADIENT;

bool gearChanging = false;
bool previousButtonMode = false;
bool previousUpShiftButton = false;
bool previousDownShiftButton = false;
// bool buttonMode = false;
bool senseModeChanges = false;
bool correctingGear = false;
bool senseUpShiftButtonChanges = false;
bool senseDownShiftButtonChanges = false;

const unsigned long debounceInterval = 250;
const unsigned long gearChangeInterval = 200;
const unsigned long correctingGearInterval = 200;
// const unsigned long gradientReadingInterval = 10;
unsigned long timeModeChanges = 0;
unsigned long timeHallEffectReadingBefore = 0;
unsigned long timeGearChanged = 0;
unsigned long timeCorrectingGear = 0;
unsigned long timeUpDownShiftButton = 0;

float gradient = 0.0f;
float pedalSpeed = 0.0f;

// gyroscope
MPU6050 mpu(Wire);

Servo servo;

void hallEffectReading();

void setup() {
  Serial.begin(115200);
  servo.attach(pinServo);
  pinMode(pinBicycleMode, INPUT_PULLUP);
  pinMode(hallEffectSensor, INPUT_PULLUP);
  // pinMode(pinBypass, INPUT_PULLUP);
  pinMode(pinUpShift, INPUT_PULLUP);
  pinMode(pinDownShift, INPUT_PULLUP);
  attachInterrupt(hallEffectSensor, hallEffectReading, FALLING);
  // attachInterrupt(pinBicycleMode, bicycleModeChanging, FALLING)
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
  bool buttonMode = digitalRead(pinBicycleMode);

  // if there is changes in bicycle mode
  if (!senseModeChanges) {
    if (buttonMode == LOW && buttonMode != previousButtonMode) {
      if (mode == AUTOMATIC_GRADIENT) mode = AUTOMATIC_SPEED;
      else if (mode == AUTOMATIC_SPEED) mode = MANUAL;
      else if (mode == MANUAL) mode = AUTOMATIC_GRADIENT;
      senseModeChanges = true;
      timeModeChanges = millis();
    }
  }
  // the mode in the next 250ms cant be changed to prevent bouncing, since before there is a change in bicycle mode
  else {
    if (millis() - timeModeChanges > debounceInterval) {
      senseModeChanges = false;
    }
  }
  previousButtonMode = buttonMode;
}

void bicycle() {
  // if the gear is not changing
  if (!gearChanging) {
    if (mode == AUTOMATIC_GRADIENT) {
      automaticGradient();
    } else if (mode == AUTOMATIC_SPEED) {
      automaticSpeed();
    } else if (mode == MANUAL) {
      manual();
    }
  }
}

void automaticGradient() {
  mpu.update();
  float gradient = mpu.getAngleX();
  int targetGearByGradient = getGearByGradient((int)gradient);
  int differenceGear = currentGear - targetGearByGradient;
  if (differenceGear > 1) action = DOWN;
  else if (differenceGear < -1) action = UP;
  else if (differenceGear == 0) action = NONE;
}

int getGearByGradient(int flooredGradient) {
  return 9 - flooredGradient;
}

void automaticSpeed() {
  if (pedalSpeed > frequencyTarget + toleranceHigherGear) action = UP;
  else if (pedalSpeed < frequencyTarget - toleranceLowerGear) action = DOWN;
  else action = NONE;
}

void manual() {
  // normal state is HIGH
  bool buttonUpShift = digitalRead(pinUpShift);
  bool buttonDownShift = digitalRead(pinDownShift);
  if (!senseDownShiftButtonChanges&&!senseUpShiftButtonChanges) {
    if (buttonUpShift == LOW && buttonUpShift != previousUpShiftButton) {
      senseUpShiftButtonChanges = true;
      timeUpDownShiftButton = millis();
    }
    else if(buttonDownShift == LOW && buttonDownShift != previousDownShiftButton){
      senseDownShiftButtonChanges = true;
      timeUpDownShiftButton = millis();
    }
  }
  // the upshift and downshift in the next 250ms cant be interrupted to prevent bouncing
  else {
    if (millis() - timeUpDownShiftButton > debounceInterval) {
      senseDownShiftButtonChanges = false;
      senseUpShiftButtonChanges = false;
    }
  }
  previousDownShiftButton = buttonDownShift;
  previousUpShiftButton = buttonUpShift;
}

void changeGear() {
  // there is a changes in gear before hence check if the delay time has exceeded
  if (gearChanging) {
    if (!correctingGear) {
      if (millis() - timeGearChanged > gearChangeInterval) {
        timeCorrectingGear = millis();
        correctingGear = true;
      }
    }
    // the gear is not being corrected
    else {
      if (millis() - timeCorrectingGear > correctingGearInterval) {
        servo.writeMicroseconds(pos);
        gearChanging = false;
        correctingGear = false;
        action = NONE;
      }
    }
  }
  // there are no changes in gears before or the delay changes in gears has exceeded, therefore allow changes in gear again

  else {
    if (action == UP) {
      // if the currect gear is 9 or more, then do nothing
      if (currentGear < maxGear) {
        currentGear++;
        // remember that array starts from 0 and gears start from 1, hence to call the position pwm from specified gear, it should subtracted by 1
        pos = gears[currentGear - 1];
        int extendedPos = pos + correctAmount;
        servo.writeMicroseconds(extendedPos);
        timeGearChanged = millis();
        gearChanging = true;
      }
    } else if (action == DOWN) {
      // if the currect gear is 1 or less, then do nothing
      if (currentGear > 1) {
        currentGear--;
        pos = gears[currentGear - 1];
        int extendedPos = pos - correctAmount / 4;
        servo.writeMicroseconds(extendedPos);
        timeGearChanged = millis();
        gearChanging = true;
      }
    } else if (action == NONE) {
      //do nothing
    }
  }
}

void resetSprocket() {
  pos = gears[startingGear];
  // the position should extended to ensure the chain fits to the gear
  int extendedPos = pos + correctAmount;
  servo.writeMicroseconds(extendedPos);
  delay(200);
  servo.writeMicroseconds(pos);
  delay(200);
  currentGear = startingGear;
}

// this function will be called when the there is a rising changes in hall effect sensor
void hallEffectReading() {
  unsigned long timeHallEffectReadingNow = millis();
  unsigned long intervalTwoHallEffectReading = timeHallEffectReadingNow - timeHallEffectReadingBefore;
  pedalSpeed = (1.0 / intervalTwoHallEffectReading) * KILO_HERTZ_TO_RPM;
  timeHallEffectReadingBefore = timeHallEffectReadingNow;
}