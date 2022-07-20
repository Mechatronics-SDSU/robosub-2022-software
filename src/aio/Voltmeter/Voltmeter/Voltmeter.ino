
float vin;
float vout;
float vmax;
const float bitvalue = 0.004688; //  5.052/1024=0.00493359
int WantedDelay = 5000;
unsigned long time_now = 0;
void setup() {

  // put your setup code here, to run once:
Serial.begin(9600);
  // put your main code here, to run repeatedly:



}

void loop() {

  if(millis() >= time_now + WantedDelay) //when time is a multiple of 5 seconds
    {
      time_now += WantedDelay; //increase value of time now by 5 seconds
      vin = analogRead(A2); 
      vout = vin*bitvalue;
      vmax = vout*(164.77/46.87);
      Serial.println(vmax);
    }

}
