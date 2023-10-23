
#define ShiftRegDataPin 1
#define ShiftRegLatchPin 2
#define ShiftRegClockPin 3


#define DRV8871_IN1 7
#define DRV8871_IN2 8

#define STBY_RUN 4
#define A_MOTORSPEED 5
#define A_RUNTIME 6

#define PROGRESSBARWIDTH 8

//#define WAITTIME 500
//#define RUNTIME 15000
#define WAITTIME 10
#define RUNTIME 15000

#define ACCEL_DECEL 30


byte progressBar[PROGRESSBARWIDTH];
int timerSetting;
int currentProgress;
int cleaningCycleCount;
int motorSpeed;
int Working;

void setup() {
  // Shift Register Setup
  // Create an array of size PROGRESSBARWIDTH
  // and init it to a bit field
  // i.e. [0] = 0b00000001
  //      [1] = 0b00000011
  // etc.
  pinMode(ShiftRegDataPin, OUTPUT);
  pinMode(ShiftRegLatchPin, OUTPUT);
  pinMode(ShiftRegClockPin, OUTPUT);
  
  // Setup DRV8871 PWM driver output pin and initialize
  pinMode(DRV8871_IN1, OUTPUT);
  pinMode(DRV8871_IN2, OUTPUT);
  digitalWrite(DRV8871_IN1, LOW);
  digitalWrite(DRV8871_IN2, LOW);

  // Standby/Run Pin, set to HIGH to run.
  pinMode(STBY_RUN, INPUT);
  
  // Run time analog input set up
  pinMode(A_RUNTIME, INPUT);
  // 3-bit (0-7) analog in resolution
  analogReadResolution(3);

  // Motor speed analog input set up
  pinMode(A_MOTORSPEED, INPUT);
  //analogReadResolution(8);

  
  // Globals initialization
  cleaningCycleCount = 10;
  Working = 0;

  int ledStatus = 1;

  for (int i=0; i<PROGRESSBARWIDTH; i++) {
    progressBar[i] = ledStatus;
    ledStatus = ledStatus | (ledStatus << 1);
  }
  setProgress(7);
  Serial.print("setup\n");
}

void setProgress(int v)
{
  currentProgress = v;
  digitalWrite(ShiftRegLatchPin, LOW);
  shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBar[v]);
  digitalWrite(ShiftRegLatchPin, HIGH);
}
void updateProgress()
{
    Serial.print("Update progress:");
    currentProgress--;
    // Wrap around
    if (currentProgress < 0) {
      currentProgress = 7;
    }
    Serial.print(currentProgress, DEC);
    Serial.print("\n");
    digitalWrite(ShiftRegLatchPin, LOW);
    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBar[currentProgress]);
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
  unsigned long currTime = millis();
  int run = digitalRead(STBY_RUN);
  run = run | digitalRead(STBY_RUN);
  Serial.print("In loop\n");
  Serial.print("Run state: \n"); Serial.print(run,DEC); Serial.print("\n");

  // Set up motor run time
  if (run == LOW) {
    Serial.print("Run is low\n");
    timerSetting = analogRead(A_RUNTIME); // 0-7
    Serial.print(timerSetting, DEC); Serial.print("\n");
    setProgress(timerSetting);
    Working = 1;
    cleaningCycleCount = (timerSetting + 1) * 7;
  }
  
  if (run == HIGH && (Working == 1)) {
    Serial.print("Run is high, working is set\n");
    Serial.print("cleaningCycleCount: "); Serial.print(cleaningCycleCount, DEC); Serial.print("\n");

    motorSpeed = analogRead(A_MOTORSPEED) * 35;
    Serial.print("Motor speed: ");  Serial.print(motorSpeed, DEC); Serial.print("\n");
  
    motorOn(DRV8871_IN1, DRV8871_IN2);
    wait(currTime, RUNTIME);
    currTime = millis();
    //Serial.print("Motor off at: "); Serial.print(currTime, DEC); Serial.print("\n");
    motorStop(DRV8871_IN1, DRV8871_IN2);
    
    currTime = millis();
    wait(currTime, WAITTIME);
    //Serial.print("Rev Motor on at: "); Serial.print(currTime, DEC); Serial.print("\n");
    
    motorOn(DRV8871_IN2, DRV8871_IN1);
    currTime = millis();
    wait(currTime, RUNTIME);
    //Serial.print("Rev Motor off at: "); Serial.print(currTime, DEC); Serial.print("\n");

    motorStop(DRV8871_IN2, DRV8871_IN1);
    currTime = millis();
    wait(currTime, WAITTIME);
    
    // Count down the cleaning cycles
    cleaningCycleCount--;

    // Decrement the progress display
    if (cleaningCycleCount % 7 == 0) {
      updateProgress();
    }
        // Finish the clean process
    if ( cleaningCycleCount < 0 ) {
      Working = 0;
    }
  }

}




void testProgressBar()
{
    for (int i = 0; i < PROGRESSBARWIDTH; i++) {
    digitalWrite(ShiftRegLatchPin, LOW);

    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBar[i]);
    digitalWrite(ShiftRegLatchPin, HIGH);
    delay(500);
  }
  // Clear all LEDs
  digitalWrite(ShiftRegLatchPin, LOW);
  shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, 0);
  digitalWrite(ShiftRegLatchPin, HIGH);
  delay(500);

  // From all lighted to none lighted
  for (int i = PROGRESSBARWIDTH-1; i >= 0; i--) {
    digitalWrite(ShiftRegLatchPin, LOW);

    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBar[i]);
    digitalWrite(ShiftRegLatchPin, HIGH);
    delay(500);
  }
// Clear all LEDs
  digitalWrite(ShiftRegLatchPin, LOW);
  shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, 0);
  digitalWrite(ShiftRegLatchPin, HIGH);
  delay(500);
}
