/**
 * masterprocess.c
 * Scion's "startup code" that starts up all other programs and utilities.
 * 
 * Contributers:
 * Ian Reichard
 */
#include "masterprocess.h"

#include <stdio.h>
#include <unistd.h>
#include <sys/wait.h>
#include <math.h>

/** helpcommand()
 * Print helpful info
 */
int helpcommand() 
{
	long unsigned int i;
	for (i = 0; i < sizeof(helparguments)/sizeof(helparguments[0]); i++) {
		printf("%s\n", helparguments[i]);
	}
	exit(EXIT_SUCCESS);
}

/** execprogram()
 * Context switches to the program, given its program integer. This function
 * shall NOT BE CALLED from the parent process, EVER. This shall ONLY be called
 * from child processes post fork().
 * 
 * prognum is the program integer derived from the -s argument
 */
void execprogram(int prognum) 
{
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

int argparse(int argc, char *argv[], argdef *argstructptr) 
{
	int i = 1;
	/*Grab all arguments*/
	for (; i < argc; i++) {
		/*Hit an argument*/
		if (*(*(argv + i)) == '-') {
			switch(*(*(argv + i) + 1)) {
				case 'a':
					argstructptr->setaarg = 1;
					break;
				case 'h':
					argstructptr->setharg = 1;
					break;
				case 'i':
					argstructptr->setiarg = 1;
					break;
				case 's':
					argstructptr->setsarg = 1;
					argstructptr->sptr = *(argv + i + 1);
					break;
				default:;
			}
		}
	}
	return 0; /*Successful parsing*/
}


int main(int argc, char *argv[]) 
{
	int i;
	int s;
	int argResult = 0;
	argdef argstruct;
	argstruct.sptr = NULL;
	argstruct.setaarg = 0;
	argstruct.setharg = 0;
	argstruct.setiarg = 0;
	argstruct.setsarg = 0;
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
	/*Test to see if help argument was issued first, if it was ignore all others*/
	if (argstruct.setharg)
		helpcommand();
	/*Comment this out to add I/O capability once that has been coded elsewhere*/
	/* if (argstruct.setiarg) */
	/*Successful parsing indicates sptr was set, convert to int*/
	if (NULL != argstruct.sptr)
		s = atoi(argstruct.sptr);
	else
		s = 0;
	/*If a was set, change s to be everything*/
	if (argstruct.setaarg)
		s = 60; /*Every other arg added together*/
	if (s < 1) { /*Don't fork, integer failed to correctly parse*/
		printf("Error: -s argument  less than 1.\n");
		exit(EXIT_FAILURE);
	}
	/*Go through everything in s argument*/
	int numPrograms = sizeof(programStartup)/sizeof(programStartup[0]);
	/*Commented out myPid line until we need it later*/
	/* pid_t myPid = getpid();*/
	pid_t childPid;
	for (i = numPrograms-1; i > 0; i--) {
		/*Bitwise AND with the powers of 2 in s argument*/
		if (s & (int)pow(2, i-1)) {
			childPid = fork();
			if (0 == childPid) {
				/*Begin child*/
				printf("PID: %d | Starting up %s\n", getpid(), *(*(programStartup + i)+1));
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

