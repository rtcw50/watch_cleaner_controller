
#define ShiftRegDataPin 1
#define ShiftRegLatchPin 2
#define ShiftRegClockPin 3

#define DRV8871_IN1 7
#define DRV8871_IN2 8

#define STBY_RUN 4

#define PROGRESSBARWIDTH 8
byte progressBar[PROGRESSBARWIDTH];

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
  int ledStatus = 1;
  for (int i=0; i<PROGRESSBARWIDTH; i++) {
    progressBar[i] = ledStatus;
    ledStatus = ledStatus | (ledStatus << 1);
  }
  resetProgress();

  // Setup DRV8871 PWM driver output pin and initialize
  pinMode(DRV8871_IN1, OUTPUT);
  pinMode(DRV8871_IN2, OUTPUT);
  digitalWrite(DRV8871_IN1, LOW);
  digitalWrite(DRV8871_IN2, LOW);

  // Standby/Run Pin, set to HIGH to run.
  pinMode(STBY_RUN, INPUT);

  Serial.print("setup\n");
}

void resetProgress(void)
{
  digitalWrite(ShiftRegLatchPin, LOW);
  shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, 0);
  digitalWrite(ShiftRegLatchPin, HIGH);
}

void ledOn(int v)
{
    digitalWrite(ShiftRegLatchPin, LOW);
    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, v);
    digitalWrite(ShiftRegLatchPin, HIGH);
}
void ledOff()
{
    digitalWrite(ShiftRegLatchPin, LOW);
    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, 0x0);
    digitalWrite(ShiftRegLatchPin, HIGH);
}

void updateProgress()
{
    static int progressCount = 0;
    Serial.print("Update progress:");
    Serial.print(progressCount, DEC);
    Serial.print("\n");
    digitalWrite(ShiftRegLatchPin, LOW);
    shiftOut(ShiftRegDataPin, ShiftRegClockPin, MSBFIRST, progressBar[progressCount]);
    digitalWrite(ShiftRegLatchPin, HIGH);
    progressCount++;
    // Wrap around
    if (progressCount >= 8) {
      progressCount = 0;
      resetProgress();
    }
}
void motorStop(int Cntrl1, int Cntrl2)
{
  // Decelerate
  digitalWrite(Cntrl2, LOW);
  for (int i = 255; i >= 0; i--) {
    analogWrite(Cntrl1, i);
    delay(20);
  }
}
void motorOn(int Cntrl1, int Cntrl2)
{
  digitalWrite(Cntrl2, LOW);
  for (int i = 0; i < 255; i++) {
    analogWrite(Cntrl1, i);
    delay(20);
  }
}

void wait(unsigned long t, unsigned int amt)
{
  unsigned long now = millis();
  while ((now - t) < amt) {
    now = millis();
  }
}
#define WAITTIME 500
#define RUNTIME 15000
void loop() {
  unsigned long currTime = millis();
  int run = digitalRead(STBY_RUN);
  run = run | digitalRead(STBY_RUN);
  //Serial.print("In loop\n");
  if (run == HIGH) {
    updateProgress();
    
    //Serial.print("In motor control section\n");
    //Serial.print("Motor on at: "); Serial.print(currTime, DEC); Serial.print("\n");
       

    motorOn(DRV8871_IN1, DRV8871_IN2);
    //ledOn(1);
    wait(currTime, RUNTIME);
    currTime = millis();
    //Serial.print("Motor off at: "); Serial.print(currTime, DEC); Serial.print("\n");
    motorStop(DRV8871_IN1, DRV8871_IN2);
    //ledOff();
    
    currTime = millis();
    wait(currTime, WAITTIME);
    //Serial.print("Rev Motor on at: "); Serial.print(currTime, DEC); Serial.print("\n");
    motorOn(DRV8871_IN2, DRV8871_IN1);
    //ledOn(2);

    currTime = millis();
    wait(currTime, RUNTIME);
    //Serial.print("Rev Motor off at: "); Serial.print(currTime, DEC); Serial.print("\n");

    motorStop(DRV8871_IN2, DRV8871_IN1);
    //ledOff();
    currTime = millis();
    wait(currTime, WAITTIME);
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


