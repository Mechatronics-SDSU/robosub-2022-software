#ifndef AIO_PACKETS_h
#define AIO_PACKETS_h

/////////////////// PINS /////////////////////

#define BAT_1_PIN         2
#define BAT_2_PIN         3      
#define MOSFET_PIN        4
#define TORPEDO_1_PIN     5
#define ARM_PIN           6
#define LEAK_1_PIN        7
#define LEAK_2_PIN        8
#define TORPEDO_2_PIN     9
#define LED_STRIP_PIN     11  
#define AUTO_PIN          A0          
#define KILL_PIN          A1

#define LED_COUNT         60          // Number of NeoPixel cells on LED strip

/////////////////// PACKETS //////////////////

#define AUTO_MASK           0x10
#define AUTO_OFF            0x10
#define AUTO_ON             0x11
#define AUTO_RX             0x1E
#define AUTO_GET            0x1F

#define BAT_MASK            0x20
#define BAT_STABLE          0x20
#define BAT_WARN_1          0x21
#define BAT_WARN_2          0x22
#define BAT_WARN_BOTH       0x23
#define BAT_RX              0x2E
#define BAT_GET             0x2F

#define KILL_MASK           0x30
#define KILL_OFF            0x30
#define KILL_ON             0x31
#define KILL_RX             0x3E
#define KILL_GET            0x3F

#define LEAK_MASK           0x40
#define LEAK_FALSE          0x40
#define LEAK_TRUE           0x41
#define LEAK_RX             0x4E
#define LEAK_GET            0x4F

#define TORPEDO_MASK        0x80
#define TORPEDO_EMPTY_1     0x81
#define TORPEDO_EMPTY_2     0x82
#define TORPEDO_EMPTY_BOTH  0x83
#define TORPEDO_FIRE_1      0x85
#define TORPEDO_FIRE_2      0x86
#define TORPEDO_FIRE_BOTH   0x87
#define TORPEDO_RX          0x8E
#define TORPEDO_GET         0x8F 

#define ARM_MASK            0xA0
#define ARM_OPEN            0xA0
#define ARM_CLOSE           0xA1
#define ARM_RX              0xAE
#define ARM_GET             0xAF

#endif