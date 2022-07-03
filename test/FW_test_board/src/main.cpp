// // Library
// #include <Arduino.h>
// #include <SPI.h>
// #include <nRF24L01.h>
// #include <RF24.h>
// #include <Servo.h>
// #include <string.h>
// #include "I2Cdev.h"
// // #include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// // Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// // is used in I2Cdev.h
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     #include "Wire.h"
// #endif
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
// // #define CE_PIN 7
// // #define CSN_PIN 8
// #define SERVO1_PIN 5
// #define SERVO2_PIN 6
// #define INTERRUPT_PIN 3
// // #define POWER_MODE RF24_PA_MIN
// // Data Structure
// struct sensor_info {
//   int16_t gyro_x;
//   int16_t gyro_y;
//   int16_t gyro_z;
//   int16_t accel_x;
//   int16_t accel_y;
//   int16_t accel_z;
//   // int tempetature;
// };
// struct sensor_packet {
//   byte change_rf_mode;
//   int16_t gyro_x;
//   int16_t gyro_y;
//   int16_t gyro_z;
//   int16_t accel_x;
//   int16_t accel_y;
//   int16_t accel_z;
//   // int temperature;
// };
// struct cam_info {
//   unsigned int cam_raw_data;
// };
// struct tx_packet {
//   byte change_rf_mode;
//   int16_t gyro_x;
//   int16_t gyro_y;
//   int16_t gyro_z;
//   int16_t accel_x;
//   int16_t accel_y;
//   int16_t accel_z;
//   // int temperature;
//   // unsigned int cam_raw_data;
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

// // MPU6050
// MPU6050 accelgyro;
// // MPU control/status vars
// bool dmpReady = false;  // set true if DMP init was successful
// uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// uint16_t fifoCount;     // count of all bytes currently in FIFO
// uint8_t fifoBuffer[64]; // FIFO storage buffer

// // orientation/motion vars
// Quaternion q;           // [w, x, y, z]         quaternion container
// VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// // Common Variable
// const byte address[6] = "00001";
// byte rf_mode;    // 0: Rx.    1: Tx


// // FROM MPU^)%)

// // // MPU6050 mpu;
// // // MPU control/status vars
// // bool dmpReady = false;  // set true if DMP init was successful
// // uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
// // uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
// // uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
// // uint16_t fifoCount;     // count of all bytes currently in FIFO
// // uint8_t fifoBuffer[64]; // FIFO storage buffer

// // // orientation/motion vars
// // Quaternion q;           // [w, x, y, z]         quaternion container
// // VectorInt16 aa;         // [x, y, z]            accel sensor measurements
// // VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
// // VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
// // VectorFloat gravity;    // [x, y, z]            gravity vector
// // float euler[3];         // [psi, theta, phi]    Euler angle container
// // float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector



// // Function
// volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//     mpuInterrupt = true;
// }


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
//   Serial.println("CTRL Receive..");
//   if (rf_mode != 0) {
//     setRF_Mode('r');
//     delay(400);
//   }
//   if (radio.available()) {
//     radio.read(&received, sizeof(ctrl_packet));
//     // packet에서 정보 추출
//     ret.pos_servo1 = received.pos_servo1;
//     ret.pos_servo2 = received.pos_servo2;
//     //받은 정보 출력
//     Serial.println("Packet Received:");
//     Serial.print("servo1: ");
//     Serial.print(ret.pos_servo1);
//     Serial.print("   |servo2: ");
//     Serial.println(ret.pos_servo2);
//     // return ret;
//   }
//   else {
//     ret.pos_servo1 = -1;
//     ret.pos_servo2 = -1;
//     Serial.println("No CTRL");
//   }
//   //모드 변환
//   if (received.change_rf_mode = 1) {
//     rf_mode = !(rf_mode);
//     setRF_Mode('t');
//     delay(400);
//   }
//   return ret;
// }
// void adjustMotor(ctrl_info data) {
//   Serial.println("Motor Adjusting..");
//   // if ((data.pos_servo1 != -1) && (data.pos_servo2 != -1)){
//     //서보모터 조정
//     // servo1.write(data.pos_servo1);
//     servo1.write(40);
//     // servo2.write(data.pos_servo2);
//     servo2.write(40);
//     Serial.print("Servo Motor1 angle:");
//     Serial.print(data.pos_servo1);
//     Serial.print("   |Servo Motor2 angle:");
//     Serial.println(data.pos_servo2);
//   // }
// }

// // #include <iostream>
// // #include <fstream>

// sensor_info getSensors() {
//   sensor_info temp;
//   int16_t ax;
//   int16_t ay;
//   int16_t az;
//   int16_t gx;
//   int16_t gy;
//   int16_t gz;
//   Serial.println("Get Sensor value from MPU6050...");
//   accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   // accelgyro.getMotion6(&temp.accel_x, &temp.accel_y, &temp.accel_z, &temp.gyro_x, &temp.gyro_y, &temp.gyro_z);
//   Serial.print("temp.accel_x :");
//   // Serial.print(temp.accel_x);
//   Serial.print(ax);
//   Serial.print(" temp.accel_y : ");
//   // Serial.print(temp.accel_y);
//   Serial.print(ay);
//   Serial.print(" temp.accel_z : ");
//   // Serial.print(temp.accel_z);
//   Serial.print(az);
//   Serial.print(" temp.gyro_x : ");
//   // Serial.print(temp.gyro_x);
//   Serial.print(gx);
//   Serial.print(" temp.gyro_y: ");
//   // Serial.print(temp.gyro_y);
//   Serial.print(gy);
//   Serial.print("  temp.gyro_z  ");
//   // Serial.println(temp.gyro_z);
//   Serial.println(gz);
//   temp.accel_x = ax;
//   temp.accel_y = ay;
//   temp.accel_z = az;
//   temp.gyro_x = gx;
//   temp.gyro_y = gy;
//   temp.gyro_z = gz;

//   accelgyro.dmpGetQuaternion(&q, fifoBuffer);
//   accelgyro.dmpGetGravity(&gravity, &q);
//   accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
//   Serial.print("ypr\t");
//   Serial.print(ypr[0] * 180/M_PI);
//   Serial.print("\t");
//   Serial.print(ypr[1] * 180/M_PI);
//   Serial.print("\t");
//   Serial.println(ypr[2] * 180/M_PI);

//   // accelgyro.dmpGetQuaternion(&q, fifoBuffer);
//   accelgyro.dmpGetAccel(&aa, fifoBuffer);
//   // accelgyro.dmpGetGravity(&gravity, &q);
//   accelgyro.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//   Serial.print("areal\t");
//   Serial.print(aaReal.x);
//   Serial.print("\t");
//   Serial.print(aaReal.y);
//   Serial.print("\t");
//   Serial.println(aaReal.z);


// // FROM MPU6050
// // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
// //               mpu.dmpGetQuaternion(&q, fifoBuffer);
// //             mpu.dmpGetAccel(&aa, fifoBuffer);
// //             mpu.dmpGetGravity(&gravity, &q);
// //             mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
// //             Serial.print("areal\t");
// //             Serial.print(aaReal.x);
// //             Serial.print("\t");
// //             Serial.print(aaReal.y);
// //             Serial.print("\t");
// //             Serial.println(aaReal.z);
// // }
//   return temp;
// }

// tx_packet add_packet(tx_packet *tx_data, sensor_info sensor_data) {
//   Serial.println("Adding Packet..");
//   tx_data->accel_x = sensor_data.accel_x;
//   tx_data->accel_y = sensor_data.accel_y;
//   tx_data->accel_z = sensor_data.accel_z;
//   tx_data->gyro_x = sensor_data.gyro_x;
//   tx_data->gyro_y = sensor_data.gyro_y;
//   tx_data->gyro_z = sensor_data.gyro_z;
//   // tx_data->temperature = sensor_data.tempetature;
// }
// tx_packet add_packet(tx_packet *tx_data, cam_info cam_data) {
//   // tx_data->cam_raw_data = cam_data.cam_raw_data;
// }
// void sendPacket(tx_packet transmit) {
//   if (rf_mode != 1) {// join I2C bus (I2Cdev library doesn't do this automatically)
//     setRF_Mode('t');
//     delay(400);
//   }
//   Serial.println("Sending Packet..");
//   transmit.change_rf_mode = 1;
//   radio.write(&transmit, sizeof(tx_packet));
//     Serial.print("temp.accel_x :");
//   Serial.print(transmit.accel_x);
//   Serial.print(" temp.accel_y : ");
//   Serial.print(transmit.accel_y);

//   Serial.print(" temp.accel_z : ");
//   Serial.print(transmit.accel_z);

//   Serial.print(" temp.gyro_x : ");
//   Serial.print(transmit.gyro_x);
  
//   Serial.print(" temp.gyro_y: ");
//   Serial.print(transmit.gyro_y);
  
//   Serial.print("  temp.gyro_z  ");
//   Serial.println(transmit.gyro_z);
//   setRF_Mode('r');

//   Serial.println("Sent:");
//   delay(400);
// }
// void sendPacket(ctrl_info transmit) {
//   ctrl_packet tr;
//   if (rf_mode != 1) {
//     setRF_Mode('t');
//     delay(400);
//   }
//   Serial.println("Sending Packet..");
//   tr.change_rf_mode = 1;
//   tr.pos_servo1 = transmit.pos_servo1;
//   tr.pos_servo2 = transmit.pos_servo2;
//   radio.write(&tr, sizeof(ctrl_packet));
//   setRF_Mode('r');
//   Serial.print("Sent:    ");
//   Serial.print("1:");
//   Serial.print(tr.pos_servo1);
//   Serial.print("    2: ");
//   Serial.println(tr.pos_servo2);
//   delay(400);
// }
//   // int servo1pos, servo2pos;
// ctrl_info getCTRLfromSerial() {
//   int servo1pos = 0;
//   int servo2pos = 0;
//   // String pos1, pos2;
//   // if (Serial.available()){
//   //   Serial.println("Servo1:");
//   //   // servo1pos = Serial.parseInt();
//   //   // servo1pos = Serial.read();
//   //   delay(100);
//   //   pos1 = Serial.readStringUntil('\n');
//   //   delay(100);
//   //   servo1pos = atoi(pos1.c_str());
//   //   Serial.println(servo1pos);
//   // }
//   // delay(1000);
//   // if (Serial.available()){
//   //   Serial.println("Servo2:");
//   //   // servo2pos = Serial.parseInt();
//   //   // servo2pos = Serial.read();
//   //   delay(100);
//   //   pos2 = Serial.readStringUntil('\n');
//   //   delay(100);
//   //   servo2pos = atoi(pos2.c_str());
//   //   Serial.println(servo2pos);
//   // }
//   Serial.println("type PRESET: ");
//   delay(200);
//   //PRESET 사용하기
//   int presetnum;
//   if (Serial.available()) {
//     delay(100);
//     presetnum = Serial.read();
//     switch(presetnum) {
//       case 'a':
//         Serial.println("1: 180, 2: 0");
//         servo1pos = 180;
//         servo2pos = 0;
//         break;
//       case 's':
//         Serial.println("1: 90, 2: 90");
//         servo1pos = 90;
//         servo2pos = 90;
//         break;
//       case 'd':
//         Serial.println("1: 0, 2: 180");
//         servo1pos = 0;
//         servo2pos = 180;
//         break;
//       default:
//         Serial.println("1: 0, 2: 0");
//         servo1pos = 0;
//         servo2pos = 0;
//         break;
//     }
//   }
//   delay(1000);
//   ctrl_info ret;
//   ret.pos_servo1 = servo1pos;
//   ret.pos_servo1 = servo2pos;
//   rf_mode = !(rf_mode);
//   setRF_Mode('t');
//   delay(200);
//   return (ret);
// }
// sensor_info receiveSensor() {
//   sensor_packet received;
//   sensor_info ret;
//   if (rf_mode != 0) {
//     setRF_Mode('r');
//     delay(200);
//   }
//   Serial.println("Receive Sensor: ");
//   if (radio.available()) {
//     radio.read(&received, sizeof(sensor_packet));
//     ret.accel_x = received.accel_x;
//     ret.accel_y = received.accel_y;
//     ret.accel_z = received.accel_z;
//     ret.gyro_x = received.gyro_x;
//     ret.gyro_y = received.gyro_y;
//     ret.gyro_z = received.gyro_z;
//     // ret.tempetature = received.temperature;
//     Serial.println("Packet Received:");
//     Serial.print("accel_x:");
//     Serial.print(received.accel_x);
//     Serial.print(" accel_y:");
//     Serial.print(received.accel_y);
//     Serial.print(" accel_z");
//     Serial.println(received.accel_z);
//     Serial.print("gyro_x:");
//     Serial.print(received.gyro_x);
//     Serial.print("gyro_y");
//     Serial.print(received.gyro_y);
//     Serial.print("gyro_z");
//     Serial.println(received.gyro_z);
//   }
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



//   // // FROM MPU6050
//   // join I2C bus (I2Cdev library doesn't do this automatically)
//     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//         Wire.begin();
//     #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//         Fastwire::setup(400, true);
//     #endif

//     // initialize serial communication
//     // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
//     // it's really up to you depending on your project)
//     // Serial.begin(9600);

//     // initialize device
//     Serial.println("Initializing I2C devices...");
//     accelgyro.initialize();
//     pinMode(INTERRUPT_PIN, INPUT);

//     // load and configure the DMP
//     Serial.println(F("Initializing DMP..."));
//     devStatus = accelgyro.dmpInitialize();
//     accelgyro.setDMPEnabled(true);
//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);

//     // set OFFSET
//     accelgyro.setXAccelOffset(-6515);
//     accelgyro.setYAccelOffset(-844);
//     accelgyro.setZAccelOffset(1419);
//     accelgyro.setXGyroOffset(-7);
//     accelgyro.setYGyroOffset(33);
//     accelgyro.setZGyroOffset(6);

//     // verify connection
//     Serial.println("Testing device connections...");
//     Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

//     // use the code below to change accel/gyro offset values
//     /*
//     Serial.println("Updating internal sensor offsets...");
//     // -76	-2359	1688	0	0	0
//     Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//     Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//     Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//     Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//     Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//     Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//     Serial.print("\n");
//     accelgyro.setXGyroOffset(220);
//     accelgyro.setYGyroOffset(76);
//     accelgyro.setZGyroOffset(-85);
//     Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//     Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//     Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//     Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//     Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//     Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//     Serial.print("\n");
//     */

//     // configure Arduino LED pin for output
//     // pinMode(LED_PIN, OUTPUT);
// }
// void loop() {
//   ctrl_info ctrl_data;
//   sensor_info sensor_data;
//   // cam_info cam_data;
//   tx_packet tx_data;
//   // 임시
//   #ifdef CANSAT
//   // ctrl_data = getCTRLfromSerial();
//   ctrl_data = getCTRL();
//   adjustMotor(ctrl_data);
//   sensor_data = getSensors();
//   //cam_data = getCam();
//   add_packet(&tx_data, sensor_data);
//   //add_packet(&tx_data, cam_data);
//   sendPacket(tx_data);
//   #endif
//   #ifdef STATION
//   ctrl_data = getCTRLfromSerial();
//   sendPacket(ctrl_data);
//   sensor_data = receiveSensor();
//   #endif
// }