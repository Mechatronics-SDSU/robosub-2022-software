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
#include <fcntl.h>
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
	FILE *fd = fopen("start/current_devices.cfg", "r");
	if (fd == NULL) {
		if (setdarg)
			printf("ERROR: current_devices.cfg does not exist.\n");
		exit(EXIT_FAILURE);
	}
	/*Read entire dev file into strings*/
	while (fgets(strbuf, buflen, fd)) {
		strbuf[strcspn(strbuf, "\n")] = 0;
		snprintf(devbuf[i], 255, "%s", strbuf);
		i++;
	}
	/*Open config with dev names*/
	FILE *fd1 = fopen("start/current_device_names.cfg", "r");
	if (fd1 == NULL) {
		if (setdarg)
			printf("ERROR: current_device_names.cfg does not exist.\n");
		exit(EXIT_FAILURE);
	}
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
			(syswdev[i])[syswdevloc[i]] = *(devs + i);
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
	int s = 0;
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
	argdef *argstructptr = &argstruct;
	/*Manual pipe allocation for watchdog*/
	int pipes[2];
	if (argstruct.setdarg)
		printf("Manually allocated pipe.\n");
	if (-1 == pipe(pipes)) { /*Pipe messed up*/
		if (argstruct.setdarg) {
			printf("Pipe creation failure\n");
			exit(EXIT_FAILURE);
		}
	}
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
		printf("%s a[%d] c[%d] d[%d] h[%d] i[%d] s[%d] w[%d]\n", "Arguments seen:",
		 argstruct.setaarg, argstruct.setcarg, argstruct.setdarg, argstruct.setharg, 
		 argstruct.setiarg, argstruct.setsarg, argstruct.setwarg);
	/*Test to see if help argument was issued first, if it was ignore all others*/
	if (argstruct.setharg)
		helpcommand();
	/*Start watchdog program with some OS magic*/
	if (argstruct.setwarg) {
		/*Establish a pipe and manually redirect stdout*/
		pid_t myPid, childPid;
		fflush(NULL);
		if (-1 == (childPid = fork())) { /*Fork messed up*/
			if (argstruct.setdarg) {
				printf("Fork failure\n");
				exit(EXIT_FAILURE);
			}
		} else if (0 == childPid) { /*Successful fork to child*/
			if (argstruct.setdarg)
				printf("Forked to child.\n");
			close(STDIN_FILENO); /*Prevent reading from stdin*/
			fflush(NULL);
			/*Redirect output*/
			if (-1 == dup2(pipes[1], STDOUT_FILENO)) {
				if (argstruct.setdarg)
					printf("dup2 failure.\n");
				exit(EXIT_FAILURE);
			}
			/*Exec watchdog*/
			execvp(wdprog[0], wdprog);
			exit(EXIT_SUCCESS);
		}
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
	if (argstruct.setdarg)
		printf("Calculated sarg=[%d]\n", s);

	/*Wait for cnc server or switch program to set config*/
	if (argstruct.setcarg) {
		/*Manual pipe allocation*/
		int cncpipes[2];
		if (-1 == pipe(cncpipes)) { /*Pipe messed up*/
			if (argstruct.setdarg) {
				printf("cnc pipe creation failure\n");
				exit(EXIT_FAILURE);
			}
		}
		pid_t myPid2, childPid2;
		fflush(NULL);
		if (-1 == (childPid2 = fork())) { /*Fork messed up*/
			if (argstruct.setdarg) {
				printf("Fork to cnc failure\n");
				exit(EXIT_FAILURE);
			}
		} else if (0 == childPid2) { /*Successful fork to child*/
			if (argstruct.setdarg)
				printf("Forked to cnc child.\n");
			close(STDIN_FILENO); /*Prevent child reading from stdin*/
			fflush(NULL);
			/*Redirect output*/
			if (-1 == dup2(cncpipes[1], STDOUT_FILENO)) {
				if (argstruct.setdarg)
					printf("dup2 failure.\n");
				exit(EXIT_FAILURE);
			}
			/*Exec cnc*/
			execvp(cncprog[0], cncprog);
		}
		/*Parent program*/
		char cbuf[255];
		cbuf[0] = '\0';
		while (1) {
			/*Read pipe for input from child*/
			read(cncpipes[0], cbuf, 255);
			/*Test for inputs to run correct programs*/
			s = atoi(cbuf);
			if (argstruct.setdarg)
				printf("Masterprocess sees CNC sent: %s\n", cbuf);
			break;
			/*Set s argument to start up relevant programs*/

		}
	}
	if (s < 1) { /*Integer failed to correctly parse at this point*/
		printf("Error: -s argument  less than 1.\n");
		exit(EXIT_FAILURE);
	}
	/*Go through everything in s argument*/
	int numPrograms = sizeof(programStartup)/sizeof(programStartup[0]);
	/*Commented out myPid line until we need it later*/
	/* pid_t myPid = getpid();*/
	pid_t childPid;
	printf("maestro subsys[2]  = %s\n", *(thrusterSubsystem + 2));
	for (i = numPrograms-1; i > 0; i--) {
		/*Bitwise AND with the powers of 2 in s argument*/
		if (s & (int)pow(2, i-1)) {
			if (argstruct.setdarg)
				printf("Seeing program at index [%ld] start up.\n", i);
			childPid = fork();
			if (0 == childPid) {
				/*Begin child*/
				if (argstruct.setdarg)
					printf("PID: %d | Starting up program: [%s %s %s]\n", 
					getpid(), *(*(programStartup + i)), *(*(programStartup + i)+1), *(*(programStartup + i)+2));
				execprogram(i);
				exit(EXIT_SUCCESS);
			}
		}
	}
	if (argstruct.setwarg) {
		char wbuf[255];
		while (1) {
			sleep(1);
			read(pipes[0], wbuf, 255);
			if (argstruct.setdarg)
				printf("Masterprocess sees: %s\n", wbuf);
		}
	}
	/*Additional functionality to be added here. For now waits for last child pid*/
	wait(&childPid);
	exit(EXIT_SUCCESS);
	return 0;
}

