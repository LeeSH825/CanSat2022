// Library
#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

// constexpr int c_strcmp( char const* lhs, char const* rhs )
// {
//     return (('\0' == lhs[0]) && ('\0' == rhs[0])) ? 0
//         :  (lhs[0] != rhs[0]) ? (lhs[0] - rhs[0])
//         : c_strcmp( lhs+1, rhs+1 );
// }

// #define STATION "station"

// #define BOARD_TYPE STATION

//#if 0 == c_strcmp( BOARD_TYPE , STATION )
// #define 

// #endif

//Flags
#define STATION
#define CANSAT

#define CE_PIN 8
#define CSN_PIN 10

#define SERVO1_PIN 3
#define SERVO2_PIN 4

#define POWER_MODE RF24_PA_MIN


// Data Structure
struct sensor_info {
  int gyro_x;
  int gyro_y;
  int gyro_z;

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

void sendData() {

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
    serial.println("Packet Received:");
    Serial.print("servo1: ");
    Serial.println(ret.pos_servo1);
    Serial.print("servo2: ");
    Serial.println(ret.pos_servo2);

    //모드 변환
    if (received.change_rf_mode = 1) {
      rf_mode = !(rf_mode);
      setRF_Mode('t');
    }
  }
}

void adjustMotor(ctrl_info data) {
  servo1.write(data.pos_servo1);
  servo2.write(data.pos_servo2);
  Serial.print("Servo Motor1 angle:")
  Serial.println(data.pos_servo1);
  Serial.print("Servo Motor1 angle:")
  Serial.println(data.pos_servo2);
}

sensor_info getSensors() {

}

tx_packet add_packet() {

}

void sendPacket(sensor_info transmit) {
  if (rf_mode != 1)
    setRF_Mode('t');
  radio.write(&transmit, sizeof(sensor_info));
}

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.setPALevel(POWER_MODE);
  #ifdef CANSAT
  // 처음 시작은 Tx모드로 
  setRF_Mode('t');
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  #endif

  #ifdef STATION
  // 처음 시작은 Rx 모드로
  setRF_Mode('r');
  #endif
  // setRF_Mode()
}
void loop() {
  ctrl_info ctrl_data;
  sensor_info sensor_data;
  tx_packet tx_data;

  ctrl_data = getCTRL();
  adjustMotor(ctrl_data);
  sensor_data = getSensors();
  //cam_data = getCam();
  tx_data = add_packet(sensor_data);
  sendPacket(tx_data);
}