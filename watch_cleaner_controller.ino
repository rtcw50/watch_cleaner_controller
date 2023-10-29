
#define ShiftRegDataPin 1
#define ShiftRegLatchPin 2
#define ShiftRegClockPin 3


#define DRV8871_IN1 7
#define DRV8871_IN2 8

#define SET_RUN 4
#define A_MOTORSPEED 5
#define A_RUNTIME 6

#define PROGRESSBARWIDTH 8

//#define WAITTIME 500
//#define RUNTIME 15000
#define WAITTIME 10
#define RUNTIME 15000
//#define RUNTIME 150

#define ACCEL_DECEL 30

#define LAST_LIT_LED_INDEX 2

// Lookup Tables indexed by A/D results
// Both A/Ds are 4-bit resolution (0-15)
// Time A/D sets the duration of the cleaning period
// Speed A/D sets the RPM of the motor 

// Time A/D Lookups
// The LED display bar bits, indexed from the timer A/D pin value
byte progressBarLookup[16] = 
  {0x0,0x0,0x1,0x1,0x3,0x3,0x7,0x7,0xF,0xF,0x1F,0x1F,0x3F,0x3F,0x7F,0xFF};
// Cleaning period duration, indexed from the timer A/D pin value
int timerSettingLookup[16] =
  {0,1,2,3,4,5,6,7,8,8,8,8,8,8,8,8 };
// Motor speed setting (PWM value) indexed from the Speed A/D pin
int motorSpeedLookup[16] = 
  {64,64,96,96,128,128,160,160,192,192,224,224,240,240,255,255};

// Value 0-15, tracks the progress of the clean, 
int currentProgress;
// The cleaning timer looked up in timerSettingLookup and indexed by the timer A/D
int timerSetting;
// The motor speed PWM value from motorSpeedLookup and indexed by the motor speed A/D
int motorSpeed;
// ncleanCycles is the number of forward/reverse spin cycles 
// timerSetting * <number>
int ncleanCycles;
// True: the motors are on, False: motors off
int Working;

int Debug=0;

void setup() {
  // Shift Register Setup
  pinMode(ShiftRegDataPin, OUTPUT);
  pinMode(ShiftRegLatchPin, OUTPUT);
  pinMode(ShiftRegClockPin, OUTPUT);
  
  // Setup DRV8871 PWM driver output pin and initialize
  pinMode(DRV8871_IN1, OUTPUT);
  pinMode(DRV8871_IN2, OUTPUT);
  digitalWrite(DRV8871_IN1, LOW);
  digitalWrite(DRV8871_IN2, LOW);

  // Standby/Run Pin, set to HIGH to run.
  pinMode(SET_RUN, INPUT);
  
  // Run time analog input set up
  pinMode(A_RUNTIME, INPUT);
  // 4-bit (0-15) analog in resolution
  analogReadResolution(4);

  // Motor speed analog input set up
  pinMode(A_MOTORSPEED, INPUT);

  
  // Globals initialization
  ncleanCycles = 5;
  Working = 0;

  setProgress(0);
  if (Debug) { Serial.print("setup\n"); }
}

void setProgress(int v)
{
  currentProgress = timerSettingLookup[v];
  digitalWrite(ShiftRegLatchPin, LOW);
  shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBarLookup[v]);
  digitalWrite(ShiftRegLatchPin, HIGH);
}

int getProgress()
{
  return currentProgress;
}

void updateProgress()
{
    if (Debug) { Serial.print("Update progress:"); }
    if (currentProgress > 0) {
      currentProgress--;
    }
    if (Debug) {
      Serial.print(currentProgress, DEC); Serial.print("\n");
    }
    digitalWrite(ShiftRegLatchPin, LOW);
    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBarLookup[currentProgress]);
    digitalWrite(ShiftRegLatchPin, HIGH);
    
}
void motorStop(int Cntrl1, int Cntrl2)
{
  // Decelerate
  digitalWrite(Cntrl2, LOW);
  for (int i = motorSpeed; i >= 0; i-=ACCEL_DECEL) {
    analogWrite(Cntrl1, i);
    delay(10);
  }
}
void motorOn(int Cntrl1, int Cntrl2)
{
  digitalWrite(Cntrl2, LOW);
  for (int i = 0; i < motorSpeed; i+=ACCEL_DECEL) {
    analogWrite(Cntrl1, i);
    delay(10);
  }
}

void wait(unsigned long t, unsigned int amt)
{
  unsigned long now = millis();
  while ((now - t) < amt) {
    now = millis();
  }
}

void loop() {
  int timerIndex, motorSpeedIndex;
  unsigned long currTime = millis();

  // Read the set/run switch
  int run = digitalRead(SET_RUN);
  delay(20);
  run = run && digitalRead(SET_RUN);
  if (Debug) {
    Serial.print("In loop\n");
    Serial.print("Run state: \n"); Serial.print(run,DEC); Serial.print("\n");
  }

  // Set up motor run time
  if (run == LOW) {
    if (Debug) { Serial.print("Run is low\n"); }
    timerIndex= analogRead(A_RUNTIME);
    timerSetting = timerSettingLookup[timerIndex];
 
    if (Debug) { Serial.print(timerSetting, DEC); Serial.print("\n"); }
    setProgress(timerIndex);
    Working = 1;
  }
  
  if (run == HIGH && (Working == 1)) {
    if (Debug) {
      Serial.print("Run is high, working is set\n");
      Serial.print("ncleanCycles: "); Serial.print(ncleanCycles, DEC); Serial.print("\n");
    }

    // clean for ncleanCycles
    for (int i=0; i<ncleanCycles; i++) {
      // Check set/run switch
      run = digitalRead(SET_RUN);
      run = run | digitalRead(SET_RUN);

      // Check set/run switch
      if (run == LOW) {
        setProgress(0);
        Working = 0;
        break;
      }

      // Update motorSpeed at the beginning of each spin cycle
      motorSpeedIndex = analogRead(A_MOTORSPEED) ; 
      motorSpeed = motorSpeedLookup[motorSpeedIndex];
      if (Debug) { 
        Serial.print("Motor speed: ");  Serial.print(motorSpeed, DEC); Serial.print("\n");
      }
  
      motorOn(DRV8871_IN1, DRV8871_IN2);
      wait(currTime, RUNTIME);
      currTime = millis();
      motorStop(DRV8871_IN1, DRV8871_IN2);
    
      currTime = millis();
      wait(currTime, WAITTIME);
    
      motorOn(DRV8871_IN2, DRV8871_IN1);
      currTime = millis();
      wait(currTime, RUNTIME);

      motorStop(DRV8871_IN2, DRV8871_IN1);
      currTime = millis();
      wait(currTime, WAITTIME);
    }
    

    // Decrement the progress display
    updateProgress();
        // Finish the clean process, when LEDs are all off (progressBar[0,1]), finish
    if ( getProgress() < LAST_LIT_LED_INDEX ) {
      Working = 0;
    }
  }

}
