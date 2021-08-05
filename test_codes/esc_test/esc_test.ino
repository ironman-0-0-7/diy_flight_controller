
void setup() {
DDRD |= B11110000;
Serial.begin(57600);

//delay(3000);
}

void loop() {


for (int i=900 ; i<2000 ; i ++ )
{
  Serial.println(i);
PORTD |= B11110000;
delayMicroseconds(i);    
PORTD &= B00001111;
delay(100);
}




 

}
