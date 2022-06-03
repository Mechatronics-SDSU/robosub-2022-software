#include "killswitch-serial.h"
#include <Adafruit_NeoPixel.h>

#define LED_PIN    11         // Which pin on the Arduino is connected to the NeoPixels?
#define LED_COUNT 60          // How many NeoPixels are attached to the Arduino?
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);      // Declare our NeoPixel strip object:

const int mosfetpin = 3;     //adjust for the number of the mosfet Pin
int buttonforce;               //stores value for piezo button tap force
int buttonsensitivity = 75;     // adjust for how sensitive the button is 
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
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP, Send the updated pixel colors to the hardware.
  strip.setBrightness(100); // Set BRIGHTNESS to about 1/5 (max = 255)
  rainbow(0);             // Flowing rainbow cycle along the whole strip for boot animation
  
  Serial.begin(9600);                       //enable serial communication connor 9600
  Serial.println("killswitch online");
}

void loop() {
  
  killswitch.serialUpdate();  //check if serial avali, set objection state.
  readbutton();
  buttontoggle();

  
  
 }//end of void loop



void readbutton(){
    buttonforce = analogRead(A1);                 //read button value and store
    //Serial.println(buttonforce);
    
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
      colorWipe(strip.Color(  0, 255,   0), 0); // Green
  }
  else{
      digitalWrite(LED_BUILTIN, LOW);                       // turn the LED off by making the voltage LOW
      digitalWrite(mosfetpin, LOW);                         // turn the LED on (HIGH is the voltage level)
      colorWipe(strip.Color(255,   0,   0), 0); // Red
  }
  }
}//end of void buttontoggle

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}//end of void colorwipe

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}
