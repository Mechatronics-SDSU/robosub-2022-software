/**
 * masterprocess.c
 * Scion's "startup code" that starts up all other programs and utilities.
 * 
 * Contributers:
 * Ian Reichard
 */
#define _DEFAULT_SOURCE
#include "masterprocess.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/wait.h>
#include <math.h>
#include <string.h>

/** helpcommand()
 * Print helpful info
 */
int helpcommand() {
	long unsigned int i;
	for (i = 0; i < sizeof(helparguments)/sizeof(helparguments[0]); i++) {
		printf("%s\n", helparguments[i]);
	}
	exit(EXIT_SUCCESS);
}

/** loaddevs()
 * Load device names from the configuration set from startup scripts.
 */
void loaddevs(int setdarg) {
	long unsigned int i = 0;
	long unsigned int j;
	int buflen = 255;
	char strbuf[buflen];
	/*Open config with devs*/
	FILE *fd = fopen("current_devices.cfg", "r");
	if (fd == NULL)
		exit(EXIT_FAILURE);
	/*Read entire dev file into strings*/
	while (fgets(strbuf, buflen, fd)) {
		strbuf[strcspn(strbuf, "\n")] = 0;
		snprintf(devbuf[i], 255, "%s", strbuf);
		i++;
	}
	/*Open config with dev names*/
	FILE *fd1 = fopen("current_device_names.cfg", "r");
	if (fd1 == NULL)
		exit(EXIT_FAILURE);
	/*Read entire dev name file into strings*/
	i = 0;
	while(fgets(strbuf, buflen, fd1)) {
		strbuf[strcspn(strbuf, "\n")] = 0;
		snprintf(devnamebuf[i], 255, "%s", strbuf);
		i++;
	}
	/*Compare dev names with index in devstrs. Rearrange and store in devs.*/
	for (i = 0; i < sizeof(devstrs)/sizeof(devstrs[0]); i++) {
		for (j = 0; j < sizeof(devnamebuf)/sizeof(devnamebuf[0]); j++) {
			if (strcmp(devstrs[i], devnamebuf[j]) == 0) {
				snprintf(devs[i], 255, "%s", devbuf[j]);
				cpydev[i] = 1;
				break;
			}
		}
	} 
	/*Change devs in syswdev with NULL pointers if 1 in cpydev*/
	
	for (i = 0; i < sizeof(syswdev)/sizeof(syswdev[0]); i++) {
		if (cpydev[i] == 1) {
			*(syswdev[i] + syswdevloc[i]) = *(devs + i);
		}
	}
	if (setdarg) {
		printf("%s\n", "Showing devices for arguments:");
		for (i = 0; i < sizeof(devs)/sizeof(devs[0]); i++) {
			printf(
			"Device name:[%s] fs location:[%s] Loaded?:[%d] Loaded dev arg:[%s]\n",
			devstrs[i], devs[i], cpydev[i], *(syswdev[i] + syswdevloc[i]));
		}
	}
}

/** execprogram()
 * Context switches to the program, given its program integer. This function
 * shall NOT BE CALLED from the parent process, EVER. This shall ONLY be called
 * from child processes post fork().
 * 
 * prognum is the program integer derived from the -s argument
 */
void execprogram(int prognum) {
	/*Exec and context switch to new program*/
	execvp(*programStartup[prognum], programStartup[prognum]);
}

/** argparse()
 * Determines program arguments.
 * argc: arg count given to main.
 * argv: argument list given to main.
 * sptr: pointer to set where the -s argument begins.
 * 
 * Returns nonzero on failure, 0 on success
 */
int argparse(int argc, char *argv[], argdef *argstructptr) {
	int i = 1;
	/*Grab all arguments*/
	for (; i < argc; i++) {
		/*Hit an argument*/
		if (*(*(argv + i)) == '-') {
			switch(*(*(argv + i) + 1)) {
				case 'a':  /*Everything*/
					argstructptr->setaarg = 1;
					break;
				case 'c':
					argstructptr->setcarg = 1;
					break;
				case 'd': /*Show output*/
					argstructptr->setdarg = 1;
					break;
				case 'h': /*Print Help*/
					argstructptr->setharg = 1;
					break;
				case 'i': /*ROS only*/
					argstructptr->setiarg = 1;
					break;
				case 's': /*Selective Startup*/
					argstructptr->setsarg = 1;
					argstructptr->sptr = *(argv + i + 1);
					break;
				case 'w': /*Start Watchdog*/
					argstructptr->setwarg = 1;
					break;
				default:;
			}
		}
	}
	return 0; /*Successful parsing*/
}

int main(int argc, char *argv[]) {
	long unsigned int i;
	int s;
	int argResult = 0;
	for (i=0;i<sizeof(cpydev)/sizeof(cpydev[0]);i++){cpydev[i]=0;}
	argdef argstruct;
	argstruct.sptr = NULL;
	argstruct.setaarg = 0;
	argstruct.setcarg = 0;
	argstruct.setdarg = 0;
	argstruct.setharg = 0;
	argstruct.setiarg = 0;
	argstruct.setsarg = 0;
	argstruct.setwarg = 0;
	FILE *wdfp;
	argdef *argstructptr = &argstruct;
	if (argc > 1)
		argResult = argparse(argc, argv, argstructptr);
	else { /*Need arguments to specify what master process does*/
		printf("Error: no arguments. Run masterprocess -h for help.\n");
		exit(EXIT_FAILURE);
	}
	if (argResult != 0) { /*Did argparse set sptr or was there an error?*/
		printf("Error: argument parsing error.\n");
		exit(EXIT_FAILURE);
	}
	if (argstruct.setdarg)
		printf("%s a[%d] d[%d] h[%d] i[%d] s[%d] w[%d]\n", "Arguments seen:",
		 argstruct.setaarg, argstruct.setdarg, argstruct.setharg, 
		 argstruct.setiarg, argstruct.setsarg, argstruct.setwarg);
	/*Test to see if help argument was issued first, if it was ignore all others*/
	if (argstruct.setharg)
		helpcommand();
	/*Start watchdog program, if applicable*/
	if (argstruct.setwarg) {
		/*Establish a pipe*/
		wdfp = popen(wdprog, "w");
		if (wdfp == NULL) /*Validate process started*/
			exit(EXIT_FAILURE);
	}
	/*Load devices from *.cfg into memory*/
	loaddevs(argstruct.setdarg);
	/*Successful parsing indicates sptr was set, convert to int*/
	if (NULL != argstruct.sptr)
		s = atoi(argstruct.sptr);
	else
		s = 0;
	/*If a was set, change s to be everything*/
	if (argstruct.setaarg)
		s = 31; /*Every other arg added together*/
	/*Test to see if we only start without API*/
	if (argstruct.setiarg)
		s = 27; /*Every arg for full autonomous without APIs*/
	if (s < 1) { /*Don't fork, integer failed to correctly parse*/
		printf("Error: -s argument  less than 1.\n");
		exit(EXIT_FAILURE);
	}
	if (argstruct.setdarg)
		printf("Calculated sarg=[%d]\n", s);
	/*Wait on watchdog for cnc server or switch program to set config*/
	if (argstruct.setcarg) {
		char buf[255];
		while (1) {
			/*Read pipe*/
			/*Test for inputs to run correct programs*/
			/*Set s argument to start up relevant programs*/
			/*Clear input*/
			fflush(wdfp);
			sleep(1);
		}
	}
	/*Go through everything in s argument*/
	int numPrograms = sizeof(programStartup)/sizeof(programStartup[0]);
	/*Commented out myPid line until we need it later*/
	/* pid_t myPid = getpid();*/
	
	pid_t childPid;
	for (i = numPrograms-1; i > 0; i--) {
		/*Bitwise AND with the powers of 2 in s argument*/
		if (s & (int)pow(2, i-1)) {
			if (argstruct.setdarg)
				printf("Seeing program at index [%ld] start up.\n", i);
			childPid = fork();
			if (0 == childPid) {
				/*Begin child*/
				if (argstruct.setdarg)
					printf("PID: %d | Starting up program: [%s %s]\n", 
					getpid(), *(*(programStartup + i)), *(*(programStartup + i)+1));
				execprogram(i);
				exit(EXIT_SUCCESS);
			}
		}
	}
	
	/*Additional functionality to be added here. For now waits for last child pid*/
	wait(&childPid);
	exit(EXIT_SUCCESS);
	return 0;
}

