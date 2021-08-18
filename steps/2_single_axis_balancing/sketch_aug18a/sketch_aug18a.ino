#include <Wire.h>

const int gyro_addr=0x68;
int temp;
int acc_reading[3];
int gyro_reading[3];


int cal_acc[3];
int cal_gyro[3];





void setup() {
Serial.begin(57600);
Serial.println("hlo world ");
gyro_setup(gyro_addr);
Serial.println("before cal acc_value");
read_mpu(gyro_addr);
Serial.println(acc_reading[2]);
cal_mpu(gyro_addr,50);
Serial.println("cal_value");
Serial.println(cal_gyro[0]);
Serial.println(cal_gyro[1]);
Serial.println(cal_gyro[2]);
Serial.println("acc_value");
read_mpu(gyro_addr);
Serial.println(gyro_reading[0]);
Serial.println(gyro_reading[1]);
Serial.println(gyro_reading[2]);
}

void loop() {
  /*
Serial.println("acc_value xx");
read_mpu(gyro_addr);
Serial.println(gyro_reading[1]);
Serial.println(gyro_reading[2]);
Serial.println(gyro_reading[0]);
delay(300);
*/
}


//####################################################################################### SET UP GYRO ######################################################################################################
void gyro_setup(int gyro_addr)
{
  //Setup the MPU-6050
    Wire.beginTransmission(gyro_addr);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_addr);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_addr);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_addr);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro
}


//################################################################################### READ MPU ###########################################################################################################
void read_mpu(int gyro_addr){
    //Read the MPU-6050
    Wire.beginTransmission(gyro_addr);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_addr,14);                                      //Request 14 bytes from the gyro.
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_reading[0] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_reading[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_reading[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temp = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_reading[0] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_reading[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_reading[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.


//SUbstrcting the initial value calculated during calibration
acc_reading[0]=acc_reading[0]-cal_acc[0];
acc_reading[1]=acc_reading[1]-cal_acc[1];
acc_reading[2]=acc_reading[2]-cal_acc[2];


gyro_reading[0]=gyro_reading[0]-cal_gyro[0];
gyro_reading[1]=gyro_reading[1]-cal_gyro[1];
gyro_reading[2]=gyro_reading[2]-cal_gyro[2];






}

//############################################################################# Calibrate MPU  ###################################################################################################################

void cal_mpu(int gyro_addr,int num_of_rounds)
{
long acc_sum[3];
long gyro_sum[3];
Serial.println("Calibrating.");

      acc_sum[0]=0;
      acc_sum[1]=0;
      acc_sum[2]=0;

      gyro_sum[0]=0;
      gyro_sum[1]=0;
      gyro_sum[2]=0;




    for (int i=0;i<num_of_rounds;i++)
    {
      Serial.print(".");
      read_mpu(gyro_addr);
      acc_sum[0]=acc_sum[0]+acc_reading[0];
      acc_sum[1]=acc_sum[1]+acc_reading[1];
      acc_sum[2]=acc_sum[2]+acc_reading[2];

      gyro_sum[0]=gyro_sum[0]+gyro_reading[0];
      gyro_sum[1]=gyro_sum[1]+gyro_reading[1];
      gyro_sum[2]=gyro_sum[2]+gyro_reading[2];
      delay(100);
    }
    Serial.println("Done ! ");
  cal_acc[0]=acc_sum[0]/num_of_rounds;
  cal_acc[1]=acc_sum[1]/num_of_rounds;
  cal_acc[2]=acc_sum[2]/num_of_rounds;

  cal_gyro[0]=gyro_sum[0]/num_of_rounds;
  cal_gyro[1]=gyro_sum[1]/num_of_rounds;
  cal_gyro[2]=gyro_sum[2]/num_of_rounds;

}
//#######################################################################################################################################################################
