/**
 * Definitions for masterprocess
 * 
 * Contributers: 
 * Ian Reichard
 */
#include <stdlib.h>

/* Struct for holding information about program's arguments */
typedef struct {
	int setaarg;
	int setcarg;
	int setdarg;
	int setharg;
	int setiarg;
	int setsarg;
	int setwarg;
	char *sptr;
}argdef;

/* Help command printout */
char *helparguments[] = {
	"Scion Master Process",
	"Arguments:",
	" -a : [a]ll, Starts up all programs + utilities.",
	" -c : [c]ommand and control, Start and wait on cnc server/autobutton."
	" -d : [d]ebug, Show full output",
	" -h : [h]elp, Prints this screen.",
	" -i : ap[i], Do not start ROS APIs, only auto programs. note: -c -w -s <auto number>.",
	" -s <number>: [s]tart, Start program(s) based on program integer.",
	" -w : [w]atchdog, Start watchdog program."
};

/*Fixed strings for devstrs in known_device_names.cfg*/
char *devstrs[] = {
	"Killswitch",
	"AHRS",
	"Depth",
	"Maestro",
	"Leak"
};

int cpydev[sizeof(devstrs)/sizeof(devstrs[0])]; /*Booleans for if something was loaded*/
char devs[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*sorted output*/
char devbuf[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*unmatched device names ex. /dev/ttyUSB1*/
char devnamebuf[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*unmatched current devstrs*/

/* Watchdog program string*/
char *wdprog = "python3 start/watchdog.py";

/* All commands are defined here in comma separated string consts for execvp 
Commands with 2 NULL pointers will have string pointers set for device names.*/
char *noop[] = {
	NULL
};
char *lsCommand[] = {
	"ls", ".", NULL
};
char *killSwitch[] = {
	"python3", "killswitch/killswitch-aio.py", NULL, NULL /*devs[0]*/
};
char *leakDetection[] = {
	"python3", "sensor/leak_detection.py", NULL, NULL /*devs[4]*/
};
char *loggingSubsystem[] = {
	"python3", "logging_sys.py", NULL
};
char *visionSubsystem[] = {
	"python3", "comms/video_client.py", NULL
};
char *sensorApi[] = {
	"rosrun", "scion_ros", "sensor_listener.py", NULL
};
char *ahrsSensor[] = {
    "rosrun", "scion_ros", "ahrs_sensor.py", NULL, NULL /*devs[1]*/
};
char *depthSensor[] = {
    "rosrun", "scion_ros", "depth_sensor.py", NULL, NULL /*devs[2]*/
};
char *sensorAPI[] = {
    "rosrun", "scion_ros", "sensor_listener.py", NULL
};
char *thrusterSubsystem[] = {
	"python3", "comms/controller_server.py", NULL, NULL  /*devs[3]*/
};
char *weaponsSubsystem[] = {
	"python3", "weapons_sys.py", NULL
};
char *heuristicsSubsystem[] = {
	"python3", "heuristics_sys.py", NULL
};
char *sensorAggSubsystem[] = {
	"python3", "sensor_agg_sys.py", NULL
};
char *trackingSubsystem[] = {
	"python3", "tracking_sys.py", NULL
};
char *detectionSubsystem[] = {
	"python3", "detection_sys.py", NULL
};

/*List of programs to change nullptrs for to device names.*/
char **syswdev[] = {
	killSwitch,
	ahrsSensor,
	depthSensor,
	thrusterSubsystem,
	leakDetection
};

/*Index location where the nullptrs are to change in syswdev.*/
int syswdevloc[] = {
	2, /*killswitch*/
	3,
	3,
	3,
	2
};

/**
* List of programs to start up, with all default arguments.
* Indexed by power of 2 in regard to the arguent chart.
*/
char **programStartup[] = {
	noop, /*Remains at start*/
	killSwitch, /*1*/
	leakDetection, /*2*/
	sensorApi, /*4*/
	ahrsSensor, /*8*/
	depthSensor, /*16*/
	thrusterSubsystem, /*32*/
	sensorAggSubsystem, /*64*/
	trackingSubsystem, /*128*/
	detectionSubsystem, /*256*/
	lsCommand /*512*/
};

int helpcommand();
void execprogram(int prognum);
int argParse();
