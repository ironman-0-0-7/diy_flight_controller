#include <Servo.h>
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input[5];

// ---------------------------------------------------------------------------
// Customize here pulse lengths as needed
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 2000 // Maximum pulse length in µs
// ---------------------------------------------------------------------------
Servo motA, motB, motC, motD;
char data;
// ---------------------------------------------------------------------------
int arm=0;
void setup() {
  
    Serial.begin(9600);
    
    motA.attach(4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    motD.attach(7, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN,LOW);

    
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.  
}


void loop() {
//

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






   
            if(arm==0)
            {
              Serial.println("DIS arm");
                     
                                  motA.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motB.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motC.writeMicroseconds(MIN_PULSE_LENGTH);
                                  motD.writeMicroseconds(MIN_PULSE_LENGTH);
            
            delay(200);
            }           

            else if(arm==1)
            {
              Serial.println(" arm !!!!!!!!");
              int pulse=receiver_input[3];
              
                                  motA.writeMicroseconds(pulse);
                                  motB.writeMicroseconds(pulse);
                                  motC.writeMicroseconds(pulse);
                                  motD.writeMicroseconds(pulse);
              
              delay(200);
              }





}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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
