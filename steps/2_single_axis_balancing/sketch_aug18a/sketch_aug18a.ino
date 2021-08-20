#include <Wire.h>
#define RAD_TO_DEG 57.295779513082320876798154814105


//complimentary filter params  YFMC c=0.9996
double c=0.998;
double c_=1-c;


const int gyro_addr=0x68;
int temp;
long acc_reading[3];
float gyro_reading[3];
/*
gyro[0] => pitch axis
gyro[1] => roll axis
gyro[2] => yaw axis
*/
int gyro_offset[3];
int frequency_of_gyro_reading=250;
// degree per second  = raw gyro /65.5
// frequency of gyro is frequency_of_gyro_reading
int conversion_constant= frequency_of_gyro_reading*65.5 ;
float gyro_yaw,gyro_roll,gyro_pitch;
double acc_roll,acc_pitch;

int pitch,roll,yaw;





//####################################################### SETUP ##############################################################################################
void setup() {
Serial.begin(57600);
Serial.println("hello world ! ! !");
Serial.println("setting up gyro");
gyro_setup(gyro_addr);//Setting up gyro registers


Serial.println("Calculating  gyro ofset");
cal_mpu(gyro_addr,50);// Calculating gyro gyro_offset



//TODO  ********************** setting initial angle value from acclerometer
gyro_yaw=0;
gyro_roll=0;
gyro_pitch=0;



/*
for(int i=0;i<10;i++)
{
  Serial.println("_______________");
  Serial.println(i);

read_mpu(gyro_addr);

 Serial.print(acc_reading[0]);
  Serial.print(" x value ||");
   Serial.print(acc_reading[1]);
    Serial.print(" y value ||");
     Serial.print(acc_reading[2]);
    Serial.print(" z value ||");

long total_accleration= (acc_reading[0]*acc_reading[0]) +( acc_reading[1]*acc_reading[1]) +( acc_reading[2]*acc_reading[2]);
total_accleration=sqrt(total_accleration);


//long total_accleration= (acc_reading[0]*acc_reading[0]);
Serial.println(total_accleration);
}

*/

/*
unsigned long start = micros();
// Call to your function
myFunction();
// Compute the time it took
unsigned long end = micros();
unsigned long delta = end - start;
Serial.println(delta);


############## calculate time of execution
*/

/*
unsigned long start = micros();
read_mpu(gyro_addr);
Serial.println(gyro_pitch);

unsigned long end_t = micros();
unsigned long delta = end_t - start;
Serial.println("delta");
Serial.println(delta);

728 micro seconds
*/
}
//####################################################################### LOOP  ################################################################################
void loop() {



/*
read_mpu(gyro_addr);
Serial.println(gyro_yaw);
delay(4);
*/
/*
read_mpu(gyro_addr);
Serial.println(gyro_pitch);
delay(4);
*/
}
//####################################################################################### SET UP GYRO ############################################################
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
//################################################################################### READ MPU ####################################################################
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


    gyro_reading[1]= Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_reading[0]= Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_reading[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.

//.......................................................... Angle estimation by intergrating gyro value .......................
//SUbstrcting the gyro_offset value calculated during calibration
gyro_reading[0]=gyro_reading[0]-gyro_offset[0];
gyro_reading[1]=gyro_reading[1]-gyro_offset[1];
gyro_reading[2]=gyro_reading[2]-gyro_offset[2];

//Converting and then intergrating gyro value
//current value = previous value + converted gyro value * dt
gyro_yaw=gyro_yaw+gyro_reading[2]/conversion_constant;
gyro_roll=gyro_roll+gyro_reading[1]/conversion_constant;
gyro_pitch=gyro_pitch+gyro_reading[0]/conversion_constant;
//...............................................................................................................................

//..................................... Angle estimation using acclerometer Reading ..............................................
long total_accleration= acc_reading[0]*acc_reading[0] + acc_reading[1]*acc_reading[1] + acc_reading[2]*acc_reading[2];
double R_acc=sqrt(total_accleration);

double x_acc=acc_reading[0];
double y_acc=acc_reading[1];

acc_pitch=asin(x_acc/R_acc);
acc_pitch=acc_pitch*RAD_TO_DEG;
acc_roll=asin(y_acc/R_acc);
acc_roll=acc_roll*RAD_TO_DEG;

// pitch=asin(inv_sin_value);
//pitch=(pitch*180)/3.14;
//...................................................................................................................................


//...................................................complimentary filter.............................................................
yaw=gyro_yaw;
roll=acc_roll*c_ + gyro_roll*c;
pitch=acc_pitch*c_+gyro_pitch*c;
//....................................................................................................................................










}
//############################################################################# Calibrate MPU  ########################################################################
void cal_mpu(int gyro_addr,int num_of_rounds)
{
long gyro_sum[3];
Serial.print("Calibrating gyro:");
      gyro_sum[0]=0;
      gyro_sum[1]=0;
      gyro_sum[2]=0;
  gyro_offset[0]=0;
  gyro_offset[1]=0;
  gyro_offset[2]=0;
    for (int i=0;i<num_of_rounds;i++)
    {
      Serial.print(".");
      read_mpu(gyro_addr);
      gyro_sum[0]=gyro_sum[0]+gyro_reading[0];
      gyro_sum[1]=gyro_sum[1]+gyro_reading[1];
      gyro_sum[2]=gyro_sum[2]+gyro_reading[2];
      delay(100);
    }
  Serial.println("Done ! ");
  gyro_offset[0]=gyro_sum[0]/num_of_rounds;
  gyro_offset[1]=gyro_sum[1]/num_of_rounds;
  gyro_offset[2]=gyro_sum[2]/num_of_rounds;

Serial.print("gyro ofset 0: ");
Serial.print(gyro_offset[0]);
Serial.print("|| gyro ofset 1: ");
Serial.print(gyro_offset[1]);
Serial.print("|| gyro ofset 2: ");
Serial.print(gyro_offset[2]);
}
//#######################################################################################################################################################################
