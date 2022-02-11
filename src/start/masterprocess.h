/**
 * Definitions for masterprocess
 * 
 * Contributers: 
 * Ian Reichard
 * /

/* Help command printout */
char *helparguments[] = {
	"Scion Master Process",
	"Arguments:",
	" -a : [a]ll, Starts up all programs + utilities",
	" -h : [h]elp, Prints this screen",
	" -i : [i]/o, Disables activation of CAN + Network",
	" -s <number>: [s]tart, Start programs based on program integer."
};

char *lsCommand[] = {
	"/bin/ls", "."
};

/**
* List of programs to start up, with all default arguments.
* Indexed by power of 2 in regard to the arguent chart.
*/
char *programStartup[] = {
	*lsCommand[0]
/*	"python3 vision_SYSTEM.py",
	"python3 sensors_SYSTEM.py",
	"python3 thrusters_SYSTEM.py",
	"python3 weapons_SYSTEM.py",
	"python3 heuristics_SYSTEM.py",
	"python3 sensor_aggregation_SYSTEM.py",
	"python3 tracking_SYSTEM.py",
	"python3 detection_SYSTEM.py"*/
};

int helpcommand();
int execprogram(int prognum);
int argParse();
