/**
 * masterprocess.c
 * Scion's "startup code" that starts up all other programs and utilities.
 * 
 * Contributers:
 * Ian Reichard
 */
#include "masterprocess.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>


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
}

/** argparse()
 * Determines program arguments.
 * 
 */

int argparse(int argc, char *argv[]) 
{

}


int main(int argc, char *argv[]) 
{
	int argResult = 0;
	if (argc > 1)
		argResult = argparse(argc, argv);
	else
		exit(EXIT_FAILURE);
	return 0;
}
