int prescaler = 256; // set this to match whatever prescaler value you set in CS registers below

// intialize values for the PWM duty cycle set by pots
float potDC1 = 0;
float potDC2 = 0;
float potDC3 = 0;
float potDC4 = 0;
float potDC5 = 0;
float potDC6 = 0;
float potDC7 = 0;
float potDC8 = 0;

void setup() {

  Serial.begin(9600);

  // input pins for valve switches
  pinMode(40, INPUT);
  pinMode(41, INPUT);
  pinMode(42, INPUT);
  pinMode(43, INPUT);
  
  pinMode(50, INPUT);
  pinMode(51, INPUT);
  pinMode(52, INPUT);
  pinMode(53, INPUT);

  // output pins for valve PWM
  pinMode(2, OUTPUT);  //OCR3B
  pinMode(3, OUTPUT);  //OCR3C
  
  pinMode(5, OUTPUT);  //OCR3A
  pinMode(6, OUTPUT);  //OCR4A
  pinMode(7, OUTPUT);  //OCR4B
  pinMode(8, OUTPUT);  //OCR4C
  
  pinMode(11, OUTPUT);  //OCR1A
  pinMode(12, OUTPUT);  //OCR1B

  int eightOnes = 255;  // this is 11111111 in binary
  TCCR1A &= ~eightOnes;  // this operation (AND plus NOT), set the eight bits in TCCR registers to 0 
  TCCR1B &= ~eightOnes;
  TCCR3A &= ~eightOnes;  
  TCCR3B &= ~eightOnes;
  TCCR4A &= ~eightOnes;
  TCCR4B &= ~eightOnes;

  // set waveform generation to frequency and phase correct, non-inverting PWM output
  TCCR1A = _BV(COM1A1)| _BV(COM1B1) ;
  TCCR1B = _BV(WGM13) | _BV(CS12);
  
  TCCR3A = _BV(COM3A1)| _BV(COM3B1) | _BV(COM3C1);
  TCCR3B = _BV(WGM33) | _BV(CS32);
  
  TCCR4A = _BV(COM4A1)| _BV(COM4B1) | _BV(COM4C1);
  TCCR4B = _BV(WGM43) | _BV(CS42);
}

void pPWM(float pwmfreq, float pwmDC1, float pwmDC2, float pwmDC3, float pwmDC4, float pwmDC5, float pwmDC6, float pwmDC7, float pwmDC8) {

  // set PWM frequency by adjusting ICR (top of triangle waveform)
  ICR1 = F_CPU / (prescaler * pwmfreq * 2);
  ICR3 = F_CPU / (prescaler * pwmfreq * 2);
  ICR4 = F_CPU / (prescaler * pwmfreq * 2);
  
  // set duty cycles
  OCR3B = (ICR3) * (pwmDC5 * 0.01);
  OCR3C = (ICR3) * (pwmDC6 * 0.01);
  OCR3A = (ICR3) * (pwmDC1 * 0.01);
  OCR4A = (ICR4) * (pwmDC2 * 0.01);
  OCR4B = (ICR4) * (pwmDC3 * 0.01);
  OCR4C = (ICR4) * (pwmDC4 * 0.01);
  OCR1A = (ICR1) * (pwmDC7 * 0.01);
  OCR1B = (ICR1) * (pwmDC8 * 0.01);
}

void loop() {

  potDC1 = 0; 
  potDC2 = 0; 
  potDC3 = 0; 
  potDC4 = 0;
  potDC5 = 0; 
  potDC6 = 0; 
  potDC7 = 0; 
  potDC8 = 0;

  // if statement for manual switch override
  if (digitalRead(40) == HIGH) { potDC1 = analogRead(A1)*100.0/1024.0; } else { potDC1 = 0.0;}
  // scale values from pot to 0 to 100, which gets used for duty cycle percentage
  if (digitalRead(41) == HIGH) { potDC2 = analogRead(A2)*100.0/1024.0; } else { potDC2 = 0.0;}
  if (digitalRead(42) == HIGH) { potDC3=  analogRead(A3)*100.0/1024.0; } else { potDC3 = 0.0;}
  if (digitalRead(43) == HIGH) { potDC4 = analogRead(A4)*100.0/1024.0; } else { potDC4 = 0.0;}
  
  if (digitalRead(50) == HIGH) { potDC5 = analogRead(A5)*100.0/1024.0; } else { potDC5 = 0.0;}
  if (digitalRead(51) == HIGH) { potDC6 = analogRead(A6)*100.0/1024.0; } else { potDC6 = 0.0;}
  if (digitalRead(52) == HIGH) { potDC7 = analogRead(A7)*100.0/1024.0; } else { potDC7 = 0.0;}
  if (digitalRead(53) == HIGH) { potDC8 = analogRead(A8)*100.0/1024.0; } else { potDC8 = 0.0;}


  float potPWMfq = 50; //set frequency to 50Hz

  // update PWM output based on the above values from pots
  pPWM(potPWMfq,potDC1,potDC2,potDC3,potDC4,potDC5,potDC6,potDC7,potDC8);

  delay(200);
}
