#include <Wire.h>
const int gyro_address=0x68;
int temperature;
int acc_axis[4], gyro_axis[4];
long acc_init_x;
long acc_init_y;
long acc_init_z;
void setup() {
  // put your setup code here, to run once:
set_gyro_registers();
 Serial.begin(57600);

acc_init_x=0;
acc_init_y=0;
acc_init_z=0;

for(int i=0;i<50;i++)
{
  gyro_signalen();
 
Serial.println(i);
  acc_init_x=acc_init_x+acc_axis[1];
  acc_init_y=acc_init_y+acc_axis[2];
  acc_init_z=acc_init_z+acc_axis[3];

  delay(300);
  }
  acc_init_x=acc_init_x/50;
  acc_init_y=acc_init_y/50;
  acc_init_z=acc_init_z/50;



Serial.println(acc_init_x);
Serial.println(acc_init_y);
Serial.println(acc_init_z);
}

void loop() {
  // put your main code here, to run repeatedly:
gyro_signalen();
int diff=acc_axis[2]-acc_init_y;

//Serial.println("Ref x:");
//Serial.println(acc_init_x);
//Serial.println("currn x:");
//Serial.println(acc_axis[1]);
//Serial.println("diff x:");
Serial.println(diff);



//Serial.println("y:");
//Serial.println(acc_axis[2-acc_init_y]);
//Serial.println("z:");
//Serial.println(acc_axis[3]-acc_init_z);

//Serial.println(gyro_axis[1]);
//Serial.println(gyro_axis[2]);
//Serial.println(gyro_axis[3]);
delay(300);
}







void gyro_signalen(){
  //Read the MPU-6050
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
//receiver_input_channel_1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
//receiver_input_channel_2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
//receiver_input_channel_3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
//receiver_input_channel_4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.





  
 /* 
  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  */

  /*
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.

*/

}



void set_gyro_registers(){
  //Setup the MPU-6050
 
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

   
}
