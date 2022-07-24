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
	" -a : [a]ll, Starts up everything. note: -c -w -s <all>",
	" -i : ap[i], Start everything except ROS APIs. note: -c -w -s <autonomous>.",
	" -c : [c]ommand and control, Start and wait on cnc server/autobutton.",
	" -d : [d]ebug, Show full output",
	" -h : [h]elp, Prints this screen.",
	" -s <number>: [s]tart, Manually start program(s) based on program integer.",
	" -w : [w]atchdog, Start watchdog program."
};

/*Fixed strings for devstrs in known_device_names.cfg*/
char *devstrs[] = {
	"AIO",
	"AHRS",
	"Depth",
	"DVL",
	"Maestro"
};

int cpydev[sizeof(devstrs)/sizeof(devstrs[0])]; /*Booleans for if something was loaded*/
char devs[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*sorted output*/
char devbuf[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*unmatched device names ex. /dev/ttyUSB1*/
char devnamebuf[sizeof(devstrs)/sizeof(devstrs[0])][255]; /*unmatched current devstrs*/

char *wdprog[] =  {"python3", "start/watchdog.py"}; /*Watchdog program*/
char *cncprog[] = {"python3", "comms/cmd_ctrl_server.py"}; /*CNC program*/

/* All commands are defined here in comma separated string consts for execvp 
Commands with 2 NULL pointers will have string pointers set for device names.*/
char *noop[] = {
	NULL
};
char *lsCommand[] = {
	"ls", ".", NULL
};
char *aioBoard[] = {
	"rosrun", "scion_ros", "aio_handler.py", NULL, NULL /*devs[0]*/
};
char *aioListener[] = {
	"rosrun", "scion_ros", "aio_listener.py", NULL	
};
char *aioForward[] = {
	"rosrun", "scion_ros", "aio_forward.py", NULL	
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
char *dvlSensor[] = {
    "rosrun", "scion_ros", "dvl_sensor.py", NULL, NULL, /*devs[3]*/
};
char *thrusterSubsystem[] = {
	"python3", "comms/controller_server.py", NULL, NULL  /*devs[4]*/
};
char *cameraDriver[] = {
	"python3", "comms/camera_scion.py", NULL
};
char *cameraMLDriver[] = {
	"python3", "ml/webcam_out.py", "ml/scion_runtime.cfg", "/mnt/SD-scion/scion_train_final.weights", "ml/scion.data", NULL
};

/*List of programs to change nullptrs for to device names.*/
char **syswdev[] = {
	aioBoard,
	ahrsSensor,
	depthSensor,
	dvlSensor,
	thrusterSubsystem
};
 
/*Index location where the nullptrs are to change in syswdev.*/
int syswdevloc[] = {
	3, /*aio*/
	3, /*ahrs*/
	3, /*depth*/
	3, /*DVL*/
	2 /*maestro*/
};

/**
* List of programs to start up, with all default arguments.
* Indexed by power of 2 in regard to the arguent chart.
*/
char **programStartup[] = {
	noop, /*Remains at start*/
	aioBoard, /*1*/
	sensorApi, /*2*/
	ahrsSensor, /*4*/
	depthSensor, /*8*/
	thrusterSubsystem, /*16*/
	cameraDriver, /*32*/
        cameraMLDriver, /*64*/
	aioForward, /*128*/
	aioListener, /*256*/
	dvlSensor, /*512*/
	lsCommand /*1024*/
};

int helpcommand();
void execprogram(int prognum);
int argParse();
