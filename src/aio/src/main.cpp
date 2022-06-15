/*Firmware for All Input Output (AIO) PCB - Rev A

  This board interfaces the following sensors for the 2022 robosub vehicle:
            Sensor           |        Packet - Message
    -------------------------|----------------------------
    Arm Gripper              |          0x0M - 0 Open, 1 Closed, F Get
    Autonomous Mode Button   |          0x1M - 0 Disable, 1 Enable, F Get
    Battery Monitor          |          0x2M - 0 Both Battery Sable,
                             |                 1 Battery #1 Voltage Unstable,
                             |                 2 Battery #2 Voltage Unstable,
                             |                 F Get 
    Kill Mode Button         |          0x3M - 0 Disable, 1 Enable, F Get
    Leak Detection           |          0x4M - 0 No Leak, 1 Leak, F Get
    LED Strip                |           n/a
    Relay Mosfet Signal      |           n/a
    Torpedo Servo Motor      |          0x8M - 1 Torpedo #1 Empty,
                             |                 2 Torpedo #2 Empty,
                             |                 3 Both Empty,
                             |                 5 Torpedo #1 Fire,
                             |                 6 Torpedo #2 Fire,
                             |                 7 Both Fire,
                             |                 F Get

  Data stream behavior:
    Interrupt Packet - A packet sent by either device indicating new alert
    ----------------------------------------------------------------------
      format: 'i'0xNM'\n'
        'i' - ascii char i header byte representing interrupt packet
        Nibblets (4bits)
          0xN_ - Type of sensor making interrupt message
          0x_M - Message value attached to sensor
        '\n' - newline byte representing newline and end of packet
    
    Output Packet - A packet sent by either device as a response to the most
                    recent interrupt packet
    ----------------------------------------------------------------------
      format: 'o'0xNM\n
        'o' - ascii char o header byte representing response packet
        Nibblets (4bits)
          0xN_ - Type of sensor making response message
          0x_M - Message value attached to sensor
        '\n' - newline end byte representing newline and end of packet
*/

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <switch.h>
#include <serial.h>
#include "aio.h"
              
const uint16_t button_sensitivity = 75;     // adjust for how sensitive the button is 
bool leak_state = 0;                        // 0 = dry, 1 = leak

unsigned long time_now = 0;
const unsigned long debounce_delay = 500;


Adafruit_NeoPixel strip(LED_COUNT, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

Switch kill_switch(KILL_PIN, button_sensitivity); // creates kill_switch object/variable that holds the boolean ON OFF
Switch auto_switch(AUTO_PIN, button_sensitivity); // creates auto_switch object/variable that holds the boolean ON OFF

Switch *kill_ptr = &kill_switch;
Switch *auto_ptr = &auto_switch;

void colorWipe(uint32_t color, int wait) {
  // Fill strip pixels one after another with a color. Strip is NOT cleared first
  // anything there will be covered pixel by pixel. Pass in color
  // as a single 'packed' 32-bit value, which you can get by calling
  // strip.Color(red, green, blue) as shown in the loop() function above),
  // and a delay time (in milliseconds) between pixels

  for(uint32_t i=0; i < strip.numPixels(); i++) { 
    strip.setPixelColor(i, color);     //  Set pixel's color (in RAM)
    strip.show();                      //  Update strip to match

    time_now = millis();               // Gather current time
    while(millis() < time_now + wait); // Delay for wait milliseconds
  }
}

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
    // Above line is equivalent to: strip.rainbow(firstPixelHue, 1, 255, 255, true);

    strip.show();                      // Update strip with new contents

    time_now = millis();               // Gather current time
    while(millis() < time_now + wait); // Delay for wait milliseconds
  }
}

void switch_update(Switch *switch_type){                                        
  if ((millis() - switch_type->_last_debounce_time) > debounce_delay){
    if (switch_type->getState() == HIGH && switch_type->_pin_num == AUTO_PIN){
      colorWipe(strip.Color(255,0,255),0);            // turn LED strip Pink
      serial_send('i',AUTO_ON);                           // Serial print AI mode enable
    }
    else if(switch_type->getState() == HIGH && switch_type->_pin_num == KILL_PIN){
      digitalWrite(LED_BUILTIN, HIGH);                // turn builtin LED on
      digitalWrite(MOSFET_PIN, HIGH);                 // turn Relay on
      colorWipe(strip.Color(0,255,0), 0);             // turn LED strip Green
      serial_send('i',KILL_OFF);                      // Serial print KILL mode disable
    }
    
    if (switch_type->getState() == LOW && switch_type->_pin_num == KILL_PIN){
      digitalWrite(LED_BUILTIN, LOW);                 // turn builtin LED off
      digitalWrite(MOSFET_PIN, LOW);                  // turn Relay off
      colorWipe(strip.Color(255,0,0), 0);             // turn LED strip Red
      serial_send('i',KILL_ON);                       // Serial print KILL mode enable
    }
  }
}

void serial_check() {
  uint8_t serial_buf = serial_listen();
  if(serial_buf != 0xFF) {
    // Perform task related to request

    if((serial_buf & ARM_MASK) == ARM_MASK){
      // Arm related task
    }

    else if((serial_buf & AUTO_MASK) == AUTO_MASK){
      // Auto related tasks
    }

    else if((serial_buf & BAT_MASK) == BAT_MASK) {
      // Battery related task
    }

    else if((serial_buf & KILL_MASK) == KILL_MASK) {
      // Kill related task
      if(serial_buf == KILL_GET) {
        if(kill_switch.getState() == HIGH) {
          serial_send('o', KILL_ON);
        }
        else if(kill_switch.getState() == LOW) {
          serial_send('o', KILL_OFF);
        }
      }
    }

    else if((serial_buf & LEAK_MASK) == LEAK_MASK) {
      // Leak related task
    }

    else if((serial_buf & TORPEDO_MASK) == TORPEDO_MASK) {
      // Torpedo related task
    }
  }
}

void leak_detection() {
  // Interrupt Service Routine for toggling leak state flag

  leak_state = 1;
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);                   
  pinMode(MOSFET_PIN, OUTPUT);
  pinMode(LEAK_PIN, INPUT);                 
  
  strip.begin();            // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();             // Turn OFF all pixels ASAP, Send the updated pixel colors to the hardware.
  strip.setBrightness(100); // Set BRIGHTNESS to about 1/5 (max = 255)
  rainbow(0);               // Flowing rainbow cycle along the whole strip for boot animation

  attachInterrupt(digitalPinToInterrupt(LEAK_PIN), leak_detection, CHANGE);
  
  Serial.begin(115200);
}

void loop() {
  // Serial Request Block
  serial_check();             // check if new message and decodes it

  // Killswitch Block
  kill_switch.readSwitch();   // check if button pressed with debounce
  switch_update(kill_ptr);    // updates switch state if press occurs

  // Autoswitch Block
  auto_switch.readSwitch();   // check if button pressed with debounce
  switch_update(auto_ptr);    // updates switch state if press occurs

  // Lead Detection Block
  if(leak_state) {
    serial_send('i',LEAK_TRUE);
    colorWipe(strip.Color(0,0,255), 0);             // turn LED strip Blue
    leak_state = 0;
  }
}