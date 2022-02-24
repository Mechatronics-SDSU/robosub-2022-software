# Scion Startup information

See the doc for a flowchart. (TODO insert here later)

### Master Process

The Master Process is used to start up most of Scion's systems.
This is done in C with the standard UNIX programming library (unistd). By performing a fork() on the initial process, we can create a parent and a child process. The parent will act like a daemon and continue to start up further processes, while the child is the subsystem running with its own instructions.

Master Process MUST be run with -s or -a arguments. If both are added, the -a argument will take precedence. If neither are added, the user is warned for lack of reading this documentation and exits.

The -a argument is a special case of the -s argument. See below how the -s argument behaves for more information.

The -h argument prints the following message:
```
Scion Master Process
Arguments:
 -a : [a]ll, Starts up all programs + utilities
 -h : [h]elp, Prints this screen
 -i : [i]/o, Disables activation of CAN + Network
 -s <number>: [s]tart, Start programs based on program integer.
```

The -i argument is currently unimplemented at the writing of this document (2/23/22). It is ignored by the program.

The -s argument is followed by an integer which specifies which programs to start up. The integer is calculated from the following table:

### Master Process Startup Argument Chart

| Integer | Subsystem |
|----------|----------|
| 1 | Logging |
| 2 | Vision Driver |
| 4 | Sensors |
| 8 | Thrusters |
| 16 | Weapons |
| 32 | Heuristics |
| 64 | Sensor Aggregation |
| 128 | Tracking |
| 256 | Detection |

To start up any number of subsystems in this table, simply add up the value in their `Integer` field in this table. This is the number passed to the master process.

For example, To run the vision driver with the thrusters, add up the 2 and the 8 to get to 10. 10 is your integer to pass to the program: `. masterprocess -s 10`

The -a argument is a special case of the -s argument. It will simply be every single one of these numbers added up, as in, to start every program. It is the equivalent of running `. masterprocess -s 511`.

