#include <PID_v1.h>

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 10;

int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
double average = 0.0;                // the average

// initialize variables for set points
double setPoint;

// initialize varibales for PID input, they will use values read from pressure sensor
double input;

// Initialize variables for output of PID control, they will apply to variables potDCn
double output;

// Set PID parameters
double Kp=0.1, Ki=10.0, Kd=0.01;

// Create PID instances
PID myPID(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);

int prescaler = 256; // set this to match whatever prescaler value you set in CS registers below

// intialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;
float potDC3 = 0;
float potDC4 = 0;

void setup() {
  // set up communication parameters
  Serial.begin(9600);

  // initialize all the readings to 0:
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  // turn on PID control
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0,255); // Set output limit

  // Hardcode setpoint
  setPoint = 8.0; // set pressure (psi)
  
  //Set up PID control parameters
  myPID.SetTunings(Kp,Ki,Kd);
  
  // input pins for valve switches
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);

  // output pins for valve PWM
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  int eightOnes = 255;  // this is 11111111 in binary
  TCCR3A &= ~eightOnes;   // this operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR3A = _BV(COM3A1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  
  TCCR4A = _BV(COM4A1) | _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}

void pPWM(float pwmfreq, float pwmDC1, float pwmDC2, float pwmDC3, float pwmDC4) {

  // set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2);
  
  // set duty cycles
  OCR3A = (ICR4) * (pwmDC1 * 0.01);
  OCR4A = (ICR4) * (pwmDC2 * 0.01);
  OCR4B = (ICR4) * (pwmDC3 * 0.01);
  OCR4C = (ICR4) * (pwmDC4 * 0.01);
}

void loop() {
  // clear settings of previous loop
  potDC1 = 0; potDC2 = 0; potDC3 = 0; potDC4 = 0;
  
  // read output voltages from sensors and convert to pressure reading in PSI, from 
  float P1 = (analogRead(A8)/1024.0 - 0.1)*100.0/0.8;
  float P2 = (analogRead(A9)/1024.0 - 0.1)*100.0/0.8;
  float P3 = (analogRead(A10)/1024.0 - 0.1)*100.0/0.8;
  float P4 = (analogRead(A11)/1024.0 - 0.1)*100.0/0.8;

  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = P1;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average = total / numReadings;

  // Give input after it is smoothed
  input = (double)average; 
  // input = P1;
  
  // Do PID calculation
  myPID.Compute();

  // if statement for manual switch override
  // scale values from pot to 0 to 100, which gets used for duty cycle percentage
  if (digitalRead(50) == HIGH) 
    { potDC1 = analogRead(A1)*100.0/1024.0; } 
  else 
    // Assume pressure depends on duty cycle settings linearly
    // Transform readings from pressure sensor to duty cylce number based on relation between pressure and duty cycles
    { potDC1 = (float)output/255 * 100.0; }
    //{ potDC1 = 50.0; }
  if (digitalRead(51) == HIGH) { potDC2 = analogRead(A2)*100.0/1024.0; }
  if (digitalRead(52) == HIGH) { potDC3 = analogRead(A3)*100.0/1024.0; }
  if (digitalRead(53) == HIGH) { potDC4 = analogRead(A4)*100.0/1024.0; }

  // Set up PWM prequency from A7 pin
   // float potPWMfq = analogRead(A7)*100.0/1024.0; // scale values from pot to 0 to 100, which gets used for frequency (Hz)
   // potPWMfq = round(potPWMfq/5)*5+1; //1 to 91 Hz in increments of 5 (rounding helps to deal with noisy pot)
  float potPWMfq = 91.0;
  
  // update PWM output based on the values calculated from PID control
  pPWM(potPWMfq,potDC1,potDC2,potDC3,potDC4);
  
  // print pressure readings
  // Serial.print(P1); Serial.print("\t");
  Serial.print(average); Serial.print("\t");
  Serial.print(setPoint); Serial.print("\n");
  //Serial.print(potDC1); Serial.print("\n");
  // Serial.print(P2); Serial.print("\t");
  // Serial.print(P3); Serial.print("\t");
  // Serial.print(P4); Serial.print("\n");

  delay(1); // Delay for better sensor reading 
}
