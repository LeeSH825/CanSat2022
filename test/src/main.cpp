// // Library
// #include <Arduino.h>
// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <Servo.h>
// #include <string.h>
// // #include <string>

// // constexpr int c_strcmp( char const* lhs, char const* rhs )
// // {
// //     return (('\0' == lhs[0]) && ('\0' == rhs[0])) ? 0
// //         :  (lhs[0] != rhs[0]) ? (lhs[0] - rhs[0])
// //         : c_strcmp( lhs+1, rhs+1 );
// // }

// // #define STATION "station"

// // #define BOARD_TYPE STATION

// //#if 0 == c_strcmp( BOARD_TYPE , STATION )
// // #define 

// // #endif

// //Flags
// // #define STATION
// #define CANSAT

// #define CE_PIN 7
// #define CSN_PIN 8

// #define SERVO1_PIN 6
// #define SERVO2_PIN 3

// #define POWER_MODE RF24_PA_MIN


// // Data Structure
// struct sensor_info {
//   int gyro_x;
//   int gyro_y;
//   int gyro_z;

//   int accel_x;
//   int accel_y;
//   int accel_z;

//   int tempetature;
// };

// struct sensor_packet {
//   byte change_rf_mode;

//   int gyro_x;
//   int gyro_y;
//   int gyro_z;

//   int accel_x;
//   int accel_y;
//   int accel_z;

//   int temperature;
// };

// struct cam_info {
//   unsigned int cam_raw_data;
// };

// struct tx_packet {
//   byte change_rf_mode;

//   int gyro_x;
//   int gyro_y;
//   int gyro_z;

//   int accel_x;
//   int accel_y;
//   int accel_z;

//   int temperature;

//   unsigned int cam_raw_data;
// };

// struct ctrl_info {
//   int pos_servo1;
//   int pos_servo2;
// };

// struct ctrl_packet {
//   byte change_rf_mode;
//   int pos_servo1;
//   int pos_servo2;
// };

// // Pin Configuration
// // RF24 radio(8, 10); // CE: Digital(8), CSN: Digital(10)
// RF24 radio(CE_PIN, CSN_PIN);
// // RF24 radio(2, 3);
// Servo servo1, servo2;

// // Common Variable
// const byte address[6] = "00001";
// byte rf_mode;    // 0: Rx.    1: Tx

// // Function

// void setRF_Mode(char mode) {
//   if (mode == 'r') {
//     radio.startListening();
//     radio.openReadingPipe(0, address);
//     rf_mode = 0;
//     Serial.println("Rx Mode");
//   }
//   else if (mode == 't') {
//     radio.stopListening();
//     radio.openWritingPipe(address);
//     rf_mode = 1;
//     Serial.println("Tx Mode");
//   }
//   else
//     Serial.println("Wrong Command!");
// }

// ctrl_info getCTRL() {
//   ctrl_packet received;
//   ctrl_info ret;

//   if (rf_mode != 0)
//     setRF_Mode('r');
//   if (radio.available()) {
//     radio.read(&received, sizeof(ctrl_packet));

//     // packet에서 정보 추출
//     ret.pos_servo1 = received.pos_servo1;
//     ret.pos_servo2 = received.pos_servo2;

//     //받은 정보 출력
//     Serial.println("Packet Received:");
//     Serial.print("servo1: ");
//     Serial.println(ret.pos_servo1);
//     Serial.print("servo2: ");
//     Serial.println(ret.pos_servo2);

<<<<<<< HEAD
//Flags
// #define STATION
// #define CANSAT

// #define CE_PIN 7
// #define CSN_PIN 8

// #define SERVO1_PIN 6
// #define SERVO2_PIN 3

// #define POWER_MODE RF24_PA_MIN
=======
//     //모드 변환
//     if (received.change_rf_mode = 1) {
//       rf_mode = !(rf_mode);
//       setRF_Mode('t');
//     }
//     //return ret;
//   }
//   return ret;
// }

// void adjustMotor(ctrl_info data) {
//   servo1.write(data.pos_servo1);
//   servo2.write(data.pos_servo2);
//   Serial.print("Servo Motor1 angle:");
//   Serial.println(data.pos_servo1);
//   Serial.print("Servo Motor2 angle:");
//   Serial.println(data.pos_servo2);
// }

// sensor_info getSensors() {
//   sensor_info temp;
//   temp.accel_x = 1;
//   temp.accel_y = 1;
//   temp.accel_z = 1;
//   temp.gyro_x = 1;
//   temp.gyro_y = 1;
//   temp.gyro_z = 1;

//   return temp;
// }
>>>>>>> 984784e (wrong)

// tx_packet add_packet(tx_packet *tx_data, sensor_info sensor_data) {
//   tx_data->accel_x = sensor_data.accel_x;
//   tx_data->accel_y = sensor_data.accel_y;
//   tx_data->accel_z = sensor_data.accel_z;
//   tx_data->gyro_x = sensor_data.gyro_x;
//   tx_data->gyro_y = sensor_data.gyro_y;
//   tx_data->gyro_z = sensor_data.gyro_z;
//   tx_data->temperature = sensor_data.tempetature;
// }

// tx_packet add_packet(tx_packet *tx_data, cam_info cam_data) {
//   tx_data->cam_raw_data = cam_data.cam_raw_data;
// }

<<<<<<< HEAD
  int accel_x;
  int accel_y;
  int accel_z;

  int tempetature;
};

struct sensor_packet {
  byte change_rf_mode;

  int gyro_x;
  int gyro_y;
  int gyro_z;

  int accel_x;
  int accel_y;
  int accel_z;

  int temperature;
};

struct cam_info {
  unsigned int cam_raw_data;
};

struct tx_packet {
  byte change_rf_mode;

  int gyro_x;
  int gyro_y;
  int gyro_z;

  int accel_x;
  int accel_y;
  int accel_z;

  int temperature;

  unsigned int cam_raw_data;
};

struct ctrl_info {
  int pos_servo1;
  int pos_servo2;
};

struct ctrl_packet {
  byte change_rf_mode;
  int pos_servo1;
  int pos_servo2;
};

// Pin Configuration
// RF24 radio(8, 10); // CE: Digital(8), CSN: Digital(10)
RF24 radio(CE_PIN, CSN_PIN);
// RF24 radio(2, 3);
Servo servo1, servo2;

// Common Variable
const byte address[6] = "00001";
byte rf_mode;    // 0: Rx.    1: Tx

// Function

void setRF_Mode(char mode) {
  if (mode == 'r') {
    radio.startListening();
    radio.openReadingPipe(0, address);
    rf_mode = 0;
    Serial.println("Rx Mode");
  }
  else if (mode == 't') {
    radio.stopListening();
    radio.openWritingPipe(address);
    rf_mode = 1;
    Serial.println("Tx Mode");
  }
  else
    Serial.println("Wrong Command!");
}

ctrl_info getCTRL() {
  ctrl_packet received;
  ctrl_info ret;

  if (rf_mode != 0)
    setRF_Mode('r');
  if (radio.available()) {
    radio.read(&received, sizeof(ctrl_packet));

    // packet에서 정보 추출
    ret.pos_servo1 = received.pos_servo1;
    ret.pos_servo2 = received.pos_servo2;

    //받은 정보 출력
    Serial.println("Packet Received:");
    Serial.print("servo1: ");
    Serial.println(ret.pos_servo1);
    Serial.print("servo2: ");
    Serial.println(ret.pos_servo2);

    //모드 변환
    if (received.change_rf_mode = 1) {
      rf_mode = !(rf_mode);
      setRF_Mode('t');
    }
    //return ret;
  }
  return ret;
}

void adjustMotor(ctrl_info data) {
  servo1.write(data.pos_servo1);
  servo2.write(data.pos_servo2);
  Serial.print("Servo Motor1 angle:");
  Serial.println(data.pos_servo1);
  Serial.print("Servo Motor2 angle:");
  Serial.println(data.pos_servo2);
}

sensor_info getSensors() {
  sensor_info temp;
  temp.accel_x = 1;
  temp.accel_y = 1;
  temp.accel_z = 1;
  temp.gyro_x = 1;
  temp.gyro_y = 1;
  temp.gyro_z = 1;

  return temp;
}

tx_packet add_packet(tx_packet *tx_data, sensor_info sensor_data) {
  tx_data->accel_x = sensor_data.accel_x;
  tx_data->accel_y = sensor_data.accel_y;
  tx_data->accel_z = sensor_data.accel_z;
  tx_data->gyro_x = sensor_data.gyro_x;
  tx_data->gyro_y = sensor_data.gyro_y;
  tx_data->gyro_z = sensor_data.gyro_z;
  tx_data->temperature = sensor_data.tempetature;
}

tx_packet add_packet(tx_packet *tx_data, cam_info cam_data) {
  tx_data->cam_raw_data = cam_data.cam_raw_data;
}

void sendPacket(tx_packet transmit) {
  if (rf_mode != 1)
    setRF_Mode('t');
  radio.write(&transmit, sizeof(tx_packet));
  setRF_Mode('r');
  Serial.println("Sent:");
}

void sendPacket(ctrl_info transmit) {
  ctrl_packet tr;
  if (rf_mode != 1)
    setRF_Mode('t');
  tr.change_rf_mode = 1;
  tr.pos_servo1 = transmit.pos_servo1;
  tr.pos_servo2 = transmit.pos_servo2;
  radio.write(&tr, sizeof(ctrl_packet));
  setRF_Mode('r');
  Serial.println("Sent:");
}

  // int servo1pos, servo2pos;
ctrl_info getCTRLfromSerial() {
  int servo1pos = 0;
  int servo2pos = 0;
  String pos1, pos2;
  if (Serial.available()){
    Serial.println("Servo1:");
    // servo1pos = Serial.parseInt();
    // servo1pos = Serial.read();
    delay(100);
    pos1 = Serial.readStringUntil('\n');
    delay(100);
    servo1pos = atoi(pos1.c_str());
    Serial.println(servo1pos);
  }
  delay(1000);
  if (Serial.available()){
    Serial.println("Servo2:");
    // servo2pos = Serial.parseInt();
    // servo2pos = Serial.read();
    delay(100);
    pos2 = Serial.readStringUntil('\n');
    delay(100);
    servo2pos = atoi(pos2.c_str());
    Serial.println(servo2pos);
  }
  delay(1000);
  ctrl_info ret;
  ret.pos_servo1 = servo1pos;
  ret.pos_servo1 = servo2pos;

  rf_mode = !(rf_mode);
  setRF_Mode('t');

  return (ret);
}

sensor_info receiveSensor() {
  sensor_packet received;
  sensor_info ret;

  if (rf_mode != 0)
    setRF_Mode('r');
  if (radio.available()) {
    radio.read(&received, sizeof(sensor_packet));

    ret.accel_x = received.accel_x;
    ret.accel_y = received.accel_y;
    ret.accel_z = received.accel_z;
    ret.gyro_x = received.gyro_x;
    ret.gyro_y = received.gyro_y;
    ret.gyro_z = received.gyro_z;
    ret.tempetature = received.temperature;

    Serial.println("Packet Received:");
    Serial.print("accel_x:");
    Serial.print(ret.accel_x);
    Serial.print(" accel_y:");
    Serial.print(ret.accel_y);
    Serial.print(" accel_z");
    Serial.println(ret.accel_z);
    
    Serial.print("gyro_x:");
    Serial.print(ret.gyro_x);
    Serial.print(" gyro_y:");
    Serial.print(ret.gyro_y);
    Serial.print(" gyro_z:");
    Serial.println(ret.gyro_z);

    Serial.print("Temp:");
    Serial.print(ret.tempetature);

    //모드 변환
    if (received.change_rf_mode = 1) {
      rf_mode = !(rf_mode);
      setRF_Mode('t');
    }
    return ret;
  }
}

void setup() {
  Serial.begin(9600);

  radio.begin();
  radio.setPALevel(POWER_MODE);
  if (radio.isChipConnected())
    Serial.println("RF Connected");
  else
    Serial.println("RF\\\\\ not Connected!!");
  #ifdef CANSAT
  // 처음 시작은 Rx모드로 
  setRF_Mode('r');
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  #endif

  #ifdef STATION
  // 처음 시작은 Tx 모드로
  setRF_Mode('t');
  #endif
  // setRF_Mode()
}



void loop() {
  ctrl_info ctrl_data;
  sensor_info sensor_data;
  cam_info cam_data;
  tx_packet tx_data;


  
  // 임시
  #ifdef CANSAT
  // ctrl_data = getCTRLfromSerial();
  ctrl_data = getCTRL();
  adjustMotor(ctrl_data);
  sensor_data = getSensors();
  //cam_data = getCam();
  add_packet(&tx_data, sensor_data);
  //add_packet(&tx_data, cam_data);
  sendPacket(tx_data);
  #endif
  #ifdef STATION
  ctrl_data = getCTRLfromSerial();
  sendPacket(ctrl_data);
  sensor_data = receiveSensor();
  #endif
}

// void serial_loop() {
=======
// void sendPacket(tx_packet transmit) {
//   if (rf_mode != 1)
//     setRF_Mode('t');
//   radio.write(&transmit, sizeof(tx_packet));
//   setRF_Mode('r');
//   Serial.println("Sent:");
// }
>>>>>>> 984784e (wrong)
//   // int servo1pos, servo2pos;
// ctrl_info getCTRLfromSerial() {
//   int servo1pos = 0;
//   int servo2pos = 0;
//   String pos1, pos2;
//   if (Serial.available()){
//     Serial.println("Servo1:");
//     // servo1pos = Serial.parseInt();
//     // servo1pos = Serial.read();
//     delay(100);
//     pos1 = Serial.readStringUntil('\n');
//     delay(100);
//     servo1pos = atoi(pos1.c_str());
//     Serial.println(servo1pos);
//   }
//   delay(1000);
//   if (Serial.available()){
//     Serial.println("Servo2:");
//     // servo2pos = Serial.parseInt();
//     // servo2pos = Serial.read();
//     delay(100);
//     pos2 = Serial.readStringUntil('\n');
//     delay(100);
//     servo2pos = atoi(pos2.c_str());
//     Serial.println(servo2pos);
//   }
//   delay(1000);
//   ctrl_info ret;
//   ret.pos_servo1 = servo1pos;
//   ret.pos_servo1 = servo2pos;

//   rf_mode = !(rf_mode);
//   setRF_Mode('t');

//   return (ret);
// }

// void setup() {
//   Serial.begin(9600);

//   radio.begin();
//   radio.setPALevel(POWER_MODE);
//   if (radio.isChipConnected())
//     Serial.println("RF Connected");
//   else
//     Serial.println("RF\\\\\ not Connected!!");
//   #ifdef CANSAT
//   // 처음 시작은 Rx모드로 
//   setRF_Mode('r');
//   servo1.attach(SERVO1_PIN);
//   servo2.attach(SERVO2_PIN);
//   #endif

//   #ifdef STATION
//   // 처음 시작은 Tx 모드로
//   setRF_Mode('t');
//   #endif
//   // setRF_Mode()
// }



// void loop() {
//   ctrl_info ctrl_data;
//   sensor_info sensor_data;
//   cam_info cam_data;
//   tx_packet tx_data;


//   // ctrl_data = getCTRL();
//   // 임시
//   ctrl_data = getCTRLfromSerial();

//   adjustMotor(ctrl_data);

//   sensor_data = getSensors();
//   //cam_data = getCam();

//   add_packet(&tx_data, sensor_data);
//   //add_packet(&tx_data, cam_data);

//   sendPacket(tx_data);
// }

// // void serial_loop() {
// //   // int servo1pos, servo2pos;
// //   if (Serial.available() > 0 ){
// //   Serial.println("Servo1:");
// //   // servo1pos = Serial.parseInt();
// //   servo1pos = Serial.read();
// //   Serial.println(servo1pos);
// //   }
// //   if (Serial.available() > 0){
// //   Serial.println("Servo2:");
// //   // servo2pos = Serial.parseInt();
// //   servo2pos = Serial.read();
// //   Serial.println(servo2pos);
// //   }
// // }






/*
  Blink

  Turns an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, MEGA and ZERO
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN is set to
  the correct LED pin independent of which board is used.
  If you want to know what pin the on-board LED is connected to on your Arduino
  model, check the Technical Specs of your board at:
  https://www.arduino.cc/en/Main/Products

  modified 8 May 2014
  by Scott Fitzgerald
  modified 2 Sep 2016
  by Arturo Guadalupi
  modified 8 Sep 2016
  by Colby Newman

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/Blink
*/
#include <Arduino.h>
// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
}