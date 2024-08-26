# RoadRunner with BunyipsLib

RoadRunner v0.5 (as of 2023) has been fully implemented into BunyipsLib, meaning that you can use RoadRunner in your code
without having to set up anything more than dependencies.<br>

Accessing tuning OpModes is as simple as extending the RoadRunnerTuner class OpMode, passing in a RoadRunnerDrive instance,
and using telemetry at runtime to pick the tuning OpMode.<br>

The tuning OpModes allow you to use RobotConfig, letting you pass in your own configurations
that you would use with BunyipsOpMode. Additional interfaces such as RoadRunnerDrive have been
implemented throughout BunyipsLib to ensure that you can use RoadRunner in your own code without
having to worry about the implementation details of different drive types.<br>

Read the wiki for more information about RoadRunner with BunyipsLib.
