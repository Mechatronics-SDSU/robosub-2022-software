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

}

/** execprogram()
 * Context switches to the program, given its program integer. This function
 * shall NOT BE CALLED from the parent process, EVER. This shall ONLY be called
 * from child processes post fork().
 * 
 * prognum is the program integer derived from the -s argument
 */
int execprogram(int prognum) 
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

int argparse(int argc, char *argv[], char *sptr) 
{
	int i = 1;
	int newarg = 0; /*bool for if we hit a -*/
	char newargc = '\0'; /*character after the -*/
	/*Grab all arguments*/
	for (; i < argc; i++) {
		/*Hit an argument*/
		if (*(*argv + i) == '-') {
			newarg = 1;
			newargc = *(*argv + i + 1);
		}
		else { 
			/*Check character*/
			if (newarg == 1) {
				/*switch for what each character does*/



				newarg = 0;
			}
		}
	}
	return 0; /*Successful parsing*/
}


int main(int argc, char *argv[]) 
{
	int i;
	int argResult = 0;
	char *sptr = NULL;
	int s = 394;
	if (argc > 1)
		argResult = argparse(argc, argv, sptr);
	else { /*Need arguments to specify what master process does*/
		printf("Error: no arguments. Run masterprocess -h for help.\n");
		exit(EXIT_FAILURE);
	}
	if (argResult != 0) { /*Did argparse set sptr or was there an error?*/
		printf("Error: argument parsing error.\n");
		exit(EXIT_FAILURE);
	}
	/*Successful parsing indicates sptr was set, convert to int*/
	//s = atoi(sptr);
	if (s < 1) { /*Don't fork, integer failed to correctly parse*/
		printf("Error: -s argument  less than 1.\n");
		exit(EXIT_FAILURE);
	}
	/*Print out a list of all programs, leaving this here commented out*/
	/*printf("%s\n", "Complete list of all programs that startup can run:");
	for (i = 1; i < sizeof(programStartup)/sizeof(programStartup[0]); i++) {
		printf("%s %s\n", *(*(programStartup + i)), 
		*(*(programStartup + i)+ 1));
	}*/
	/*Perform bitwise AND until we go through everything*/
	int numPrograms = sizeof(programStartup)/sizeof(programStartup[0]);
	for (i = numPrograms-1; i > 0; i--) {
		//printf("%d\n", (int)pow(2, i-1));
		if (s & (int)pow(2, i-1)) {
			//fork
			//execvp if child
			//printf("Starting up %s\n", *(*(programStartup + i)+1));
		}
	}
	/*Fork and exec when we have a new program to run*/
	/*AFTER ARG PARSE IS DONE UNCOMMENT*/
	/*execprogram(s);*/

	exit(EXIT_SUCCESS);
	return 0;
}

