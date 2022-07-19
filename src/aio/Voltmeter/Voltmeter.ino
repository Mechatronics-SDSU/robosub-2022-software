float vin;
float vout;
float vmax;
const float bitvalue = 0.004688; //  5.052/1024=0.00493359
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);


}

void loop() {
  // put your main code here, to run repeatedly:
  vin = analogRead(A2); 
  vout = vin*bitvalue;
  vmax = vout*(164.77/46.87);
  //vmax = vout*4.46724147; //4.467 value is resistor relationship of 5.8k and 25.91k
  Serial.println(vmax);
 

}
