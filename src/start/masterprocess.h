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
	int setharg;
	int setiarg;
	int setsarg;
	char *sptr;
}argdef;

/* Holds strings for device names */
typedef struct {
	char[256] devname;
}extdev;


/* Help command printout */
char *helparguments[] = {
	"Scion Master Process",
	"Arguments:",
	" -a : [a]ll, Starts up all programs + utilities",
	" -h : [h]elp, Prints this screen",
	" -i : [i]/o, Disables activation of CAN + Network",
	" -s <number>: [s]tart, Start programs based on program integer."
};

/* All commands are defined here in comma separated string consts for execvp */
char *lsCommand[] = {
	"ls", ".", NULL
};
char *loggingSubsystem[] = {
	"python3", "logging_sys.py", NULL
};
char *visionSubsystem[] = {
	"python3", "comms/video_client.py", NULL /*devname*/
};
char *ahrsSensor[] = {
    "rosrun", "scion_ros", "ahrs_sensor.py", NULL /*devname*/
};
char *depthSensor[] = {
    "rosrun", "scion_ros", "depth_sensor.py", NULL /*devname*/
};
char *sensorAPI[] = {
    "rosrun", "scion_ros", "sensor_listener.py", NULL /*devname*/
};
char *thrusterSubsystem[] = {
	"python3", "comms/controller_server.py", "/dev/ttyACM0", NULL  /*devname*/
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

/**
* List of programs to start up, with all default arguments.
* Indexed by power of 2 in regard to the arguent chart.
*/
char **programStartup[] = {
	loggingSubsystem, /* 1 */
	visionSubsystem, /* 2 */
	sensorAPI, /* 4 */
	ahrsSensor, /* 8 */
	depthSensor, /* 16 */
	thrusterSubsystem, /* 32 */
	sensorAggSubsystem, /* 64 */
	trackingSubsystem, /* 128 */
	detectionSubsystem /* 256 */
};

int helpcommand();
void execprogram(int prognum);
int argParse();
