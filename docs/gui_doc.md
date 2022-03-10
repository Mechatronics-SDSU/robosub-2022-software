# GUI Code Documentation

### Planned Features

Scion has a variety of different sensors than Pico did. One of the major planned features for the updated GUI is a new list of sensors and preipherals.

#### Feature 1: Improved status UI for thrusters/leak detector
The Pico GUI had 6 bars to show direction of thrusters along with a %max RPM bar that would be full speed when full. This status UI shall become a picture of Scion with its 4 vectored thrusters, to give a far better indicator of how the thrusters are turning and affecting bearing. This also serves a second purpose, to map out leak detection. Mapped images of compromised leak areas shall be white if they are uncompromised, and red if they are compromised (leak was detected).

#### Feature 2: Logging to Disk
The Pico GUI only printed out logs sent to the logging area in the GUI. The updated GUI shall support logging to I/O either in some tempoorary buffer or performing I/O operations each time a log is sent.

#### Feature 3: Multiple Cameras
Scion shall support multiple cameras, and the ability to switch between them in the GUI is another requirement.

#### Feature 4: Updating program startup
With the master process completed, a new startup sequence on Scion requires a brand new approach to starting up programs with the GUI. The GUI going forward shall, for connection:
a. Set the IP to Scion's IP and attempt to contact Scion with a timeout.
b. Once a connection is made, a prompt is issued for what programs on Scion to start up. This in the backend will be checkboxes (not mutually exclusive with one another) Which shall add up to the master process's integer table. This integer is what is sent to Scion and will tell the master process what to start up.

Design:
Check which systems you would like to start:

[] Logging
[] Vision Driver
[] Sensors
[] Thrusters
[] Weapons
[] Heuristics
[] Sensor Aggregation
[] Tracking
[] Detection

#### Feature 5: Functional Kill Button, Autonomous Button, Restart Computer Button
a. The Kill button shall function as if the hardware kill button were pushed.
b. The Autonomous button shall function as if the autonomous button were pushed with the additional feature of ceasing manual control immediately. This will require consierable changes.
c. The restart computer button will provide a way to remotely reboot Scion's computer from the GUI, thus terminating any connection(s) to Scion and needing to reconnect once the computer has rebooted.

#### Feature 6: Weapons Control System
The weapons control system shall feature a way to manually fire weapons from the GUI (likely from the controller), as long as displaying the WCB state.
