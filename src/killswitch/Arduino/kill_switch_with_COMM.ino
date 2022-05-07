#include <killswitch-serial.h>

const int mosfetpin = 3;     //adjust for the number of the mosfet Pin
int buttonforce;               //stores value for piezo button tap force
int buttonsensitivity = 10;     // adjust for how sensitive the button is 
//boolean piezostate = LOW;       //stores value for current button state, starts at LOW.

unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 500;

Killswitch killswitch; //creates killswitch object/variable that connor made that holds the boolean ON OFF
                       //declaration of killswitch object

//boolean piezostate* = LOW;       //store adddress with pointer
boolean *piezostate = &killswitch._killswitch_state;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);               //declare led pin
  pinMode(mosfetpin, OUTPUT);                 //declare mosfet pin
  Serial.begin(9600);                       //enable serial communication connor 9600
}

void loop() {
  
  killswitch.serialUpdate();  //check if serial avali, set objection state.
  readbutton();
  buttontoggle();
  
 }//end of void loop



void readbutton(){
    buttonforce = analogRead(A1);                 //read button value and store
    Serial.println(buttonforce);
}


void buttontoggle(){                                        //with debounce
  if ((millis() - lastDebounceTime) > debounceDelay){
  if (buttonforce >= buttonsensitivity){                    //determine if button is pressed
    *piezostate = !*piezostate;                               //flip value of buttonstate
    lastDebounceTime = millis();                            //reset last button timer
  }
    if (*piezostate == HIGH){
      digitalWrite(LED_BUILTIN, HIGH);                      // turn the LED on (HIGH is the voltage level)
      digitalWrite(mosfetpin, HIGH);                        // turn the LED on (HIGH is the voltage level)
    }
    else{
      digitalWrite(LED_BUILTIN, LOW);                       // turn the LED off by making the voltage LOW
      digitalWrite(mosfetpin, LOW);                         // turn the LED on (HIGH is the voltage level)
    }
   }
}//end of void buttontoggle
