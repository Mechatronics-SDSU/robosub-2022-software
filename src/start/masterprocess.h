/**
 * Definitions for masterprocess
 * 
 * Contributers: 
 * Ian Reichard
 * /

/* Help command printout */
char *helparguments[] = {
	"Scion Startup Program",
	"Arguments:",
	" -a : [a]ll, Starts up all programs + utilities",
	" -h : [h]elp, Prints this screen",
	" -i : [i]/o, Disables activation of CAN + Network",
	" -s <number>: [s]tart, Start programs based on program integer."
};

int helpcommand();
int execprogram(int prognum);
int argParse();
