# Scion Startup information

See the doc for a flowchart. (TODO insert here later)

Master Process is invoked with -s or -a arguments (mutually exclusive).
For help with the master process, run `sudo masterprocess -h`.

# Master Process Startup Argument Chart

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
