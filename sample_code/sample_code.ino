int prescaler = 256; // set this to match whatever prescaler value you set in CS registers below

// intialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;
float potDC3 = 0;
float potDC4 = 0;

void setup() {

  Serial.begin(9600);

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

  potDC1 = 0; potDC2 = 0; potDC3 = 0; potDC4 = 0;

  // if statement for manual switch override
  if (digitalRead(50) == HIGH) {
  potDC1 = analogRead(A1)*100.0/1024.0; // scale values from pot to 0 to 100, which gets used for duty cycle percentage
  }
 
  if (digitalRead(51) == HIGH) { potDC2 = analogRead(A2)*100.0/1024.0; }
  if (digitalRead(52) == HIGH) { potDC3 = analogRead(A3)*100.0/1024.0; }
  if (digitalRead(53) == HIGH) { potDC4 = analogRead(A4)*100.0/1024.0; }

  float potPWMfq = analogRead(A7)*100.0/1024.0; // scale values from pot to 0 to 100, which gets used for frequency (Hz)
  // potPWMfq = round(potPWMfq/5)*5+1; //1 to 91 Hz in increments of 5 (rounding helps to deal with noisy pot)
  potPWMfq = 50; //set frequency to 50Hz

  // update PWM output based on the above values from pots
  pPWM(potPWMfq,potDC1,potDC2,potDC3,potDC4);

  // transfer function for sensor Honeywell ASDXRRX100PGAA5 (100 psi, 5V, A-calibration)
  // Vout = 0.8*Vsupply/(Pmax - Pmin)*(Papplied - Pmin) + 0.1*Vsupply
  // Rearrange to get: Papplied = (Vout/Vsupply - 0.1)*(Pmax - Pmin)/0.8 + Pmin;

  // read output voltages from sensors and convert to pressure reading in PSI
  float P1 = (analogRead(A8)/1024.0 - 0.1)*100.0/0.8;
  float P2 = (analogRead(A9)/1024.0 - 0.1)*100.0/0.8;
  float P3 = (analogRead(A10)/1024.0 - 0.1)*100.0/0.8;
  float P4 = (analogRead(A11)/1024.0 - 0.1)*100.0/0.8;

  // print pressure readings
  Serial.print(P1); Serial.print("\t");
  Serial.print(P2); Serial.print("\t");
  Serial.print(P3); Serial.print("\t");
  Serial.print(P4); Serial.print("\n");

  delay(200);
}
