
#include <SoftwareSerial.h>
#include <ODriveArduino.h>

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

// Serial to the ODrive
// Serial3: pin 15 and 14 on Arduino  
// Serial2: pin 17 and 16 on Arduino
//SoftwareSerial odrive_serial(15, 14); //RX (ODrive TX GPIO 1), TX (ODrive RX GPIO 2)
// ODrive object
//ODriveArduino odrive(odrive_serial);
ODriveArduino odrive(Serial3);
ODriveArduino odrive_1(Serial2);

void setup() {
  // ODrive uses 115200 baud
//  odrive_serial.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(115200);

  // Serial to PC
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  // In this example we set the same parameters to both motors.
  // You can of course set them different if you want.
  // See the documentation or play around in odrivetool to see the available parameters
  for (int axis = 0; axis < 2; ++axis) {
    Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 100000.0f << '\n';
    Serial3 << "w axis" << axis << ".motor.config.current_lim " << 10.0f << '\n';
    Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 100000.0f << '\n';
    Serial2 << "w axis" << axis << ".motor.config.current_lim " << 10.0f << '\n';
//    
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }

  Serial.println("Ready!");
  Serial.println("Send the character '0' or '1' to calibrate respective motor (you must do this before you can command movement)");
  Serial.println("Send the character 's' to exectue test move");
  Serial.println("Send the character 'b' to read bus voltage");
  Serial.println("Send the character 'p' to read motor positions in a 10s loop");
}

void loop() {

  if (Serial.available()) {
    char c = Serial.read();

    // Run calibration sequence
    if (c == '0' || c == '1') {
      int motornum = c-'0';
      int requested_state;
      
      // Motor calibration for odrive board 0
      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Board 0 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);
      delay(1000); //Delay 1 s
      
      // Motor calibration for odrive board 1
      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
      Serial << "Board 1 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive_1.run_state(motornum, requested_state, true);
      delay(1000); //Delay 1 s

      // Encoder calibration for odrive board 0
      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Board 0 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, true);
      delay(10000); //Delay 10 s

      // Encoder calibration for odrive board 1
      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
      Serial << "Board 1 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive_1.run_state(motornum, requested_state, true);
      delay(10000); //Delay 10 s
      
      // close loop control for odrive board 0
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Board 0 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive.run_state(motornum, requested_state, false); // don't wait

      // close loop control for odrive board 1
      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
      Serial << "Board 1 Axis" << c << ": Requesting state " << requested_state << '\n';
      odrive_1.run_state(motornum, requested_state, false); // don't wait
    }

    if (c == 'i') {
      int requested_state;
      requested_state = ODriveArduino::AXIS_STATE_IDLE;
      
      // Motor idle for odrive board 0 axis 0
      Serial << "Board 0 Axis 0: Requesting state " << requested_state << '\n';
      odrive.run_state(0, requested_state, true);
      delay(500); //Delay 500ms

      // Motor idle for odrive board 0 axis 1
      Serial << "Board 0 Axis 1: Requesting state " << requested_state << '\n';
      odrive.run_state(1, requested_state, true);
      delay(500); //Delay 500ms

      // Motor idle for odrive board 1 axis 0
      Serial << "Board 1 Axis 0: Requesting state " << requested_state << '\n';
      odrive_1.run_state(0, requested_state, true);
      delay(500); //Delay 500ms

      // Motor idle for odrive board 1 axis 1
      Serial << "Board 1 Axis 1: Requesting state " << requested_state << '\n';
      odrive_1.run_state(1, requested_state, true);
      delay(500); //Delay 500ms
    }
    // Sinusoidal test move
    if (c == 's') {
      Serial.println("Executing test move");
      for (float ph = 0.0f; ph < 6.28318530718f; ph += 0.01f) {
        float pos_m0 = 20000.0f * cos(ph);
        float pos_m1 = 20000.0f * sin(ph);
        odrive.SetPosition(0, pos_m0);
        odrive.SetPosition(1, pos_m1);
        odrive_1.SetPosition(0, pos_m0);
        odrive_1.SetPosition(1, pos_m1);
        delay(50);
      }
    }

    // Read bus voltage
    if (c == 'b') {
//      odrive_serial << "r vbus_voltage\n";

      Serial3 << "r vbus_voltage\n";     
      Serial << "Board 0 Vbus voltage: " << odrive.readFloat() << '\n';
      delay(500);
      Serial2 << "r vbus_voltage\n";
      Serial << "Board 1 Vbus voltage: " << odrive_1.readFloat() << '\n';
      
    }

    // print motor positions in a 10s loop
    if (c == 'p') {
      static const unsigned long duration = 10000;
      unsigned long start = millis();
      while(millis() - start < duration) {
        for (int motor = 0; motor < 2; ++motor) {
//          odrive_serial << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial3 << "r axis" << motor << ".encoder.pos_estimate\n";
          Serial << odrive.readFloat() << '\t';
        }
        Serial << '\n';
      }
    }
  }
}
