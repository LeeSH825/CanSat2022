// #include <ctime>
// #include <iostream>
// #include <string>
// #include <time.h>
// #include "/usr/local/include/RF24/RF24.h"

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
//   uint8_t change_rf_mode;

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
//   uint8_t change_rf_mode;

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
//   uint8_t change_rf_mode;
//   int pos_servo1;
//   int pos_servo2;
// };

// RF24 radio(24,26);
// uint8_t rf_mode;
// uint8_t address[6] = "00001";

// using namespace std;



// void setRF_Mode(char mode) {
//   if (mode == 'r') {
//     radio.startListening();
//     radio.openReadingPipe(0, address);
//     rf_mode = 0;
//     // Serial.println("Rx Mode");
//     cout << "Rx Mode";
//   }
//   else if (mode == 't') {
//     radio.stopListening();
//     radio.openWritingPipe(address);
//     rf_mode = 1;
//     // Serial.println("Tx Mode");
//     cout << "Tx Mode";
//   }
//   else
//     ;// Serial.println("Wrong Command!");
// }

// ctrl_info setCTRL() {
//   int pos_servo1, pos_servo2;
//   ctrl_info ret;

//   cout << "Enter position of servo1:";
//   cin >> pos_servo1;
//   ret.pos_servo1 = pos_servo1;
//   cout << "Enter position of servo2:";
//   cin >> pos_servo2;
//   ret.pos_servo2 = pos_servo2;

//   return ret;
// }

// void sendPacket(ctrl_info info) {
//   ctrl_packet tx_data;

//   tx_data.change_rf_mode = 1;
//   tx_data.pos_servo1 = info.pos_servo1;
//   tx_data.pos_servo2 = info.pos_servo2;
//   if (rf_mode != 1)
//     setRF_Mode('t');
//   radio.write(&tx_data, sizeof(ctrl_packet));
//   setRF_Mode('r');
// }

// sensor_info getSensor() {
//   tx_packet received;
//   sensor_info ret;
//   if (rf_mode != 0)
//     setRF_Mode('r');
//   if (radio.available()) {
//     radio.read(&received, sizeof(tx_packet));

//     ret.accel_x = received.accel_x;
//     ret.accel_y = received.accel_y;
//     ret.accel_z = received.accel_z;
//     ret.gyro_x = received.gyro_x;
//     ret.gyro_y = received.gyro_y;
//     ret.gyro_z = received.gyro_z;
//   }

//   return ret;
// }

// int main() {
// 	if(!radio.begin()){
// 		cout << "radio not responding!" << endl;
// 		return 0;
// 	}

// 	radio.setPALevel(POWER_MODE);
// 	setRF_Mode('t');
//   	rf_mode = 1;

//   	ctrl_info ctrl_data;
//   	sensor_info sensor_data;
//   	cam_info cam_data;
//   	tx_packet tx_data;

// 	while(true) {
//     	ctrl_data = setCTRL();
//     	sendPacket(ctrl_data);
//     	sensor_data = getSensor();

// 		// ?????? ????????? ctrl_data??? ????????? ????????? ???
// 		// ??????????????? ??????
// 		// ?????? ????????? ????????? ????????? ?????????
// 	}
// }