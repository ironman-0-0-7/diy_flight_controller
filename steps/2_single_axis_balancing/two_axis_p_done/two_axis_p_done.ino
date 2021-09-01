#include <Wire.h>
#define RAD_TO_DEG 57.295779513082320876798154814105

#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs

#include <Servo.h>
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input[5];


int motor_a_pulse=0;
int motor_b_pulse=0;
int motor_c_pulse=0;
int motor_d_pulse=0;

//complimentary filter params  YFMC c=0.9996
double c=0.98;
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

double pitch,roll,yaw;
double desired_pitch,desired_roll,desired_yaw;

// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
char data;
int arm=0;
// ---------------------------------------------------------------------------



//####################################################### SETUP ##############################################################################################
void setup() {
Serial.begin(57600);
Serial.println("hello world ! ! !");
Serial.println("setting up gyro");
gyro_setup(gyro_addr);//Setting up gyro registers
//.................. GYRO OFSET
//Serial.println("Calculating  gyro ofset");
//cal_mpu(gyro_addr,50);// Calculating gyro gyro_offset
gyro_offset[0]=10;
gyro_offset[1]=-123;
gyro_offset[2]=14;
//..................Setting initial value from accelorometer
read_mpu(gyro_addr);
gyro_yaw=0;
gyro_roll=acc_roll;
gyro_pitch=acc_pitch;

//.................Desired roll pitch yaw
desired_roll=0;
desired_pitch=0;
desired_yaw=0;

//................. Attaching mototrs

   motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);
//........................ Reciver input interupt
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.



/*
############## calculate time of execution
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
//unsigned long start = micros();


//Serial.print("arm value ");
//Serial.print(arm);
//Serial.println(" ");

//................................... ARM _ DISARM  ....................

if(receiver_input[1]<1100 && receiver_input[2]<1100&&receiver_input[3]<1100&&receiver_input[4]>1800)
{
  //arm
  arm=1;
  //Serial.println("arm");
  digitalWrite(LED_BUILTIN,HIGH);
  }
if(receiver_input[2]<1100 && receiver_input[4]<1100&&receiver_input[3]<1100&&receiver_input[1]>1800)
{
  //disarm
  arm=0;
  //Serial.println("DIS arm");
  digitalWrite(LED_BUILTIN,LOW);
  }
//......................................................................


//****************************************************************************IF ARMED READ GYRO ESTIMATE PID AND SEND PULSE ELSE SEND MIN PULSE
if(arm==1)
{

//STEP 1 : COPY PULSE FROM RX INPUT
motor_a_pulse=receiver_input[3];
motor_b_pulse=receiver_input[3];
motor_c_pulse=receiver_input[3];
motor_d_pulse=receiver_input[3];
//STEP 2 : READ GYRO
read_mpu(gyro_addr);
//STEP 3 :ESTIMATE  MOTOR PULSE  USING PID
calculate_motor_pulse_PID();
//STEP 4 : CHECK IF PULSE IS WITHIN RANGE
                              //.......................................................................CHECK IF PULSE IS BETWEEN 1000 AND 2000
                              if(motor_a_pulse>2000)
                              {motor_a_pulse=2000; }
                               else if (motor_a_pulse<1000)
                              {motor_a_pulse=1000;}


                              if(motor_b_pulse>2000)
                              {motor_b_pulse=2000; }
                               else if (motor_b_pulse<1000)
                              {motor_b_pulse=1000;}


                              if(motor_c_pulse>2000)
                              {motor_c_pulse=2000; }
                               else if (motor_c_pulse<1000)
                              {motor_c_pulse=1000;}


                              if(motor_d_pulse>2000)
                              {motor_d_pulse=2000; }
                              else if (motor_d_pulse<1000)
                              {motor_d_pulse=1000;}
                              //..................................................................................................................
//STEP 5 : SEND PULSE
motA.writeMicroseconds(motor_a_pulse);
motB.writeMicroseconds(motor_b_pulse);
motC.writeMicroseconds(motor_c_pulse);
motD.writeMicroseconds(motor_d_pulse);

Serial.print("Motor a  ");
Serial.print(motor_a_pulse);
Serial.print("|Motor b ");
Serial.print(motor_b_pulse);
Serial.print("|Motor c ");
Serial.print(motor_c_pulse);
Serial.print("|Motor d ");
Serial.print(motor_d_pulse);
Serial.println(" ");

}
else
{

                                  //NOT ARMES SO SEND MIN PULSE
                                  motA.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motB.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motC.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motD.writeMicroseconds(MIN_PULSE_LENGTH);

Serial.print("Motor a  ");
Serial.print(MIN_PULSE_LENGTH);
Serial.print("|Motor b ");
Serial.print(MIN_PULSE_LENGTH);
Serial.print("|Motor c ");
Serial.print(MIN_PULSE_LENGTH);
Serial.print("|Motor d ");
Serial.print(MIN_PULSE_LENGTH);
Serial.println(" ");

  }
//......................................................................................................................................................
//Serial.println(receiver_input[3]);

/*
read_mpu(gyro_addr);
Serial.println(gyro_yaw);
delay(4);
*/

/*

TIME ESTIMATION

unsigned long end_t = micros();
unsigned long delta = end_t - start;
Serial.print("delta:");
Serial.println(delta);

*/

//int delay_time =4ms
delay(3);
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
    while(Wire.available() < 14)
    {
    //while loop running
    Serial.println("INF while loop Prob !! ");
    //Serial.println(Wire.available());
    Wire.beginTransmission(gyro_addr);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_addr,14);
      }                                           //Wait until the 14 bytes are received.
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
acc_pitch=-1*acc_pitch*RAD_TO_DEG;
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
 void calculate_motor_pulse_PID()
 {
   double P_gain,I_gain,D_gain;//P gain for roll and pitch
   P_gain=15;
   I_gain=0;
   D_gain=0;
double roll_error,pitch_error,yaw_error;

 roll_error=desired_roll - roll;
 pitch_error=desired_pitch-pitch;
 yaw_error=desired_yaw-yaw;


//................................... roll pulse  ..................................
/*
M_A,M_C down => +ve roll
M_B,M_D down => -ve roll
so
error >0
=>
desired value is more
so more roll req
so MA MC should go down
MB MD should go up
so MA MC - error
MB MD +  error
*/

double roll_correction=P_gain*roll_error ;
motor_a_pulse=motor_a_pulse +  roll_correction;
motor_b_pulse=motor_b_pulse -  roll_correction;
motor_c_pulse=motor_c_pulse + roll_correction;
motor_d_pulse=motor_d_pulse -  roll_correction;

//..................................... Pitch Pulse ....................................
double pitch_correction=P_gain*pitch_error ;
/*
desired value +ve
Pitch down
so
Ma MB --
MC MD ++

*/
motor_a_pulse=motor_a_pulse - pitch_correction;
motor_b_pulse=motor_b_pulse -  pitch_correction;
motor_c_pulse=motor_c_pulse + pitch_correction;
motor_d_pulse=motor_d_pulse + pitch_correction;
 }



//############################################################################################
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 5=========================================
  if(PINB & B0001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}
