# All in One (AIO) PCB
Firmware and Python driver for communication with AIO PCB

Contents
* [What is AIO?](https://github.com/Mechatronics-SDSU/robosub-2022-software/tree/aio-dev/src/aio#what-is-aio)
* [Sensor Suite](https://github.com/Mechatronics-SDSU/robosub-2022-software/tree/aio-dev/src/aio#Sensor-Suite)
* [Packet Frame](https://github.com/Mechatronics-SDSU/robosub-2022-software/tree/aio-dev/src/aio#Packet-Frame)
* [Usage](https://github.com/Mechatronics-SDSU/robosub-2022-software/tree/aio-dev/src/aio#Usage)
* [Packet Messages](https://github.com/Mechatronics-SDSU/robosub-2022-software/tree/aio-dev/src/aio#Packet-Messages)

## What is AIO?
The all in one PCB is a centralized PCB for SDSU Mechatronics' event based sensor suite for 2022 Robosub vehicle

## Sensor Suite
- [x] Kill Switch
- [x] Autonomous Switch
- [x] Leak Detection
- [x] Battery Undervoltage Monitor
- [ ] Servo Based Torpedo Launcher
- [ ] Netwon Gripper
- [ ] System LED Strip

## Packet Frame
### Types of Frames and Format
| Header of Packet | Message | End of Packet |
|---|---|---|
| i | 1 Byte | \n |
| o | 1 Byte | \n |

### Description of Header Types
* Interrupt Header
  * Begins with an ascii 'i' to indicate the packet is an interrupt
  * These packets represent an asyncronous message that is sent from either device on the serial connection
  * Sent by AIO PCB
    * Represents an event has just occured on the AIO PCB and is relaying the information to the Main CPU
    * Example
      * The AIO PCB detected the kill button has been pressed and sends an 'i' packet to the Main CPU to inform it about the kill state change
  * Sent by Main CPU
    * Represents instructing the AIO PCB to perform an event
    * Examples
      * The Main CPU wishes to fire a torpedo and sends an 'i' packet to the AIO PCB to launch the desire torpedo
      * The Main CPU  wishes to get the current kill state and sends an 'i' packet to the AIO PCB to respond with the current kill state
* Output Header
  * Begins with an ascii 'o' to indicate the packet is an output response
  * These packets represent an asyncronous message that is sent only as a response to the previously sent 'i' message from the opposite device on the serial connection
  * Sent by AIO PCB
    * Represents a response to a querry message or ordered state change from the Main CPU
    * Example
      * The AIO PCB received an 'i' packet to change kill state from the Main CPU. It will perform the desired state change and respond with an 'o' packet containing the new kill state
  * Sent by Main CPU
    * Represents a receive confirmation to an interrupt message from AIO PCB
    * Example
      * The Main CPU received an 'i' packet representing an autonomous state change from the AIO PCB. It will respond with an 'o' packet containing a receive confirmation

## Usage
To connect to the AIO PCB and begin communication:

* You must establish a serial connection to the USB COM port or /dev/tty port
* Send a 'i' packet with ascii/utf-8 encoding over the serial connection
* Wait for 'o' packet to confirm accurate response

```
import serial
import time

# Hard coded port for serial connection
arduino = serial.Serial(port='/dev/ttyUSB1', baudrate=9600, timeout=.1)

def write_read(x):
    arduino.write(x)
    data = arduino.readline()
    return data

while True:
    command = input("Enter any key to initiate ")
    value = write_read(b'i\x3F\n') # Hard coded message sent to AIO, currently set as "get kill status" 
    print(value)
```

Responds with the following ascii decimal value if kill state is off:
```
1114810
```
* 111 is decimal for 'o' in ascii
* 48 is decimal for 0x30 in ascii which is the KILL_OFF message
* 10 is decimal for '\n' in ascii
  
Responds with the following ascii decimal value if kill state is on:
```
1114910
```
* 111 is decimal for 'o' in ascii
* 49 is decimal for 0x31 in ascii which is the KILL_ON message
* 10 is decimal for '\n' in ascii


## Packet Messages
| Name | Description | Value |
|---|---|---|
| AUTO_OFF | Indicates autonomous state is valued as off | 0x10 |
| AUTO_ON  | Indicates autonomous state is valued as on | 0x11 |
| AUTO_RX | Indicates the previous autonomous command was received | 0x1E |
| AUTO_GET | Querry for current autonomous state from PCB | 0x1F |
| BAT_STABLE | Indicates both LiPo batteries are operating at safe voltage levels | 0x20 |
| BAT_WARN_1 | Indicates LiPo battery #1 is undervoltage | 0x21 |
| BAT_WARN_2 | Indicates LiPo battery #2 is undervoltage | 0x22 |
| BAT_WARN_BOTH | Indicates both Lipo batteries are operating at unsafe voltage levels | 0x2F |
| BAT_RX | Indicates the previous battery command was received | 0x2E |
| BAT_GET | Querry for current state of both LiPo batteries | 0x2F |
| KILL_OFF | Indicates kill state is valued as off | 0x30 |
| KILL_ON | Indicates kill state is values as on | 0x31 |
| KILL_RX | Indicates the previous kill command was received | 0x3E |
| KILL_GET | Querry for current kill state from PCB | 0x3F |
| LEAK_FALSE | Indicates currently there is no leak in the vehicle | 0x40 |
| LEAK_TRUE | Indicates currently there is a leak in the vehicle | 0x41 |
| LEAK_RX | Indicates the previous leak command was received | 0x4E |
| LEAK_GET | Querry for current leak state from PCB | 0x4F |
| TORPEDO_EMPTY_1 | Indicates currently torpedo #1 has been fired | 0x81  |
| TORPEDO_EMPTY_2 | Indicates currently torpedo #2 has been fired | 0x82 |
| TORPEDO_EMPTY_BOTH | Indicates currently both torpedos have been fired | 0x83|
| TORPEDO_FIRE_1 | Instruct PCB to fire torpedo #1 | 0x85 |
| TORPEDO_FIRE_2 | Instruct PCB to fire torpedo #2 | 0x86 |
| TORPEDO_FIRE_BOTH | Instruct PCB to fire both torpedos at once | 0x87 |
| TORPEDO_RX | Indicates the previous torpedo command was received | 0x8E |
| TORPEDO_GET | Querry for current state of both torpedos | 0x8F |
| ARM_OPEN | Indicates arm state is valued as open | 0xA0 |
| ARM_CLOSE | Indicates arm state is values as closed | 0xA1 |
| ARM_RX | Indicates the previous arm command was received | 0xAE |
| ARM_GET | Querry for current arm state | 0xAF |